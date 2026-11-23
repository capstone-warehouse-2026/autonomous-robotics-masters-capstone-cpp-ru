"""Фиксирует таблицы сценариев, обученные веса и эталон Python/C++ без test selection."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from dqn import Network
from env import DispatchEnvironment, FAMILIES, make_scenario
from spec import masked_argmax, observation


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, allow_nan=False) + '\n')


def export_scenarios(destination):
    entries = []
    for split, seeds in (('train', range(3)), ('validation', range(1000, 1010)),
                         ('test', range(2000, 2020))):
        for family in FAMILIES:
            for seed in seeds:
                scenario = make_scenario(split, seed, family)
                relative = Path(split) / f'{family}_{seed}.json'
                write_json(destination / relative, scenario)
                entries.append({'path': str(relative), 'split': split,
                                'sha256': hashlib.sha256(
                                    (destination / relative).read_bytes()).hexdigest()})
    write_json(destination / 'manifest.json', {'schema_version': 1, 'scenarios': entries})


def golden_cases(model):
    cases = []
    for family in FAMILIES:
        env = DispatchEnvironment(make_scenario('validation', 1000, family))
        for _ in range(8):
            inputs, mask = env.observe()
            values = model.predict(inputs)
            action = masked_argmax(values, mask)
            cases.append({'observation': inputs.tolist(), 'mask': mask.tolist(),
                          'q_values': values.tolist(), 'action': action})
            if env.done:
                break
            env.step(action)
    return cases


def preprocessing_cases():
    cases = []
    for index, family in enumerate(FAMILIES):
        scenario = make_scenario('validation', 1000 + index, family)
        env = DispatchEnvironment(scenario)
        env.now = 40.0 * index
        env.navigation_failures = index
        env.release_jobs()
        inputs, mask = env.observe()
        robot = scenario['stations'][env.position]
        cases.append({
            'scenario': scenario, 'current_station': robot['id'],
            'context': {'now': env.now, 'robot_x': robot['x'], 'robot_y': robot['y'],
                        'loaded': False, 'navigation_failures': env.navigation_failures},
            'candidate_ids': [job['id'] for job in env.candidates()],
            'observation': inputs.tolist(), 'mask': mask.tolist(),
            'classical_action': env.classical_action(),
            'congestion_feature': True,
        })
    # Отдельно проверяем ablation и невозможность выбора при грузе.
    for loaded, congestion in ((True, True), (False, False)):
        scenario = make_scenario('validation', 1000, 'bottleneck')
        env = DispatchEnvironment(scenario)
        env.now = 100.0
        env.release_jobs()
        robot = scenario['stations'][0]
        candidates = [env.estimates(job) for job in env.candidates()]
        inputs, mask = observation(candidates, env.now, robot['x'], robot['y'],
                                   loaded=loaded, congestion=congestion)
        cases.append({'scenario': scenario, 'current_station': robot['id'],
                      'context': {'now': env.now, 'robot_x': robot['x'], 'robot_y': robot['y'],
                                  'loaded': loaded, 'navigation_failures': 0},
                      'candidate_ids': [job['id'] for job in env.candidates()],
                      'observation': inputs.tolist(), 'mask': mask.tolist(),
                      'classical_action': env.classical_action(),
                      'congestion_feature': congestion})
    # Пустая очередь допускает только ожидание.
    empty = make_scenario('validation', 1000, 'low')
    for job in empty['jobs']:
        job['release_time'] += 10.0
    env = DispatchEnvironment(empty)
    robot = empty['stations'][0]
    inputs, mask = env.observe()
    cases.append({'scenario': empty, 'current_station': robot['id'],
                  'context': {'now': 0.0, 'robot_x': robot['x'], 'robot_y': robot['y'],
                              'loaded': False, 'navigation_failures': 0},
                  'candidate_ids': [], 'observation': inputs.tolist(), 'mask': mask.tolist(),
                  'classical_action': 4, 'congestion_feature': True})
    return cases


def release_models(training, destination, ablation=None):
    report = json.loads((training / 'training.json').read_text())
    selected = max(report['seeds'], key=lambda row: row['validation_return'])
    destination.mkdir(parents=True, exist_ok=True)
    for row in report['seeds']:
        seed = row['training_seed']
        shutil.copyfile(training / f'seed_{seed}' / 'model.json',
                        destination / f'dqn_seed_{seed}.json')
        shutil.copyfile(training / f'seed_{seed}' / 'learning.csv',
                        destination / f'learning_seed_{seed}.csv')
    shutil.copyfile(training / f"seed_{selected['training_seed']}" / 'model.json',
                    destination / 'dispatch_dqn.json')
    model = Network.load(destination / 'dispatch_dqn.json')
    write_json(destination / 'golden.json', {'tolerance': 1e-4, 'cases': golden_cases(model)})
    write_json(destination / 'preprocessing_golden.json',
               {'tolerance': 1e-6, 'cases': preprocessing_cases()})
    card = {'schema_version': 1, 'format': 'JSON float32 MLP, row-major weights, ReLU hidden',
            'architecture': [32, 64, 5], 'selected_seed': selected['training_seed'],
            'selection': report['selection'], 'training': report,
            'source': 'ml/mission/{spec,env,dqn,train,export}.py',
            'limitations': ['Procedural travel tables, not full Webots navigation.',
                            'Closed-loop domain gap must be measured separately.',
                            'Training performed on the documented dirty development checkout.'],
            'model_sha256': hashlib.sha256(
                (destination / 'dispatch_dqn.json').read_bytes()).hexdigest()}
    source = Path(__file__).parent
    card['source_sha256'] = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(source.glob('*.py'))
    }
    card['all_model_sha256'] = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(destination.glob('dqn_seed_*.json'))
    }
    if ablation:
        ablation_report = json.loads((ablation / 'training.json').read_text())
        ablation_selected = max(ablation_report['seeds'], key=lambda row: row['validation_return'])
        for row in ablation_report['seeds']:
            seed = row['training_seed']
            shutil.copyfile(ablation / f'seed_{seed}' / 'model.json',
                            destination / f'dqn_no_congestion_seed_{seed}.json')
            shutil.copyfile(ablation / f'seed_{seed}' / 'learning.csv',
                            destination / f'learning_no_congestion_seed_{seed}.csv')
        shutil.copyfile(ablation / f"seed_{ablation_selected['training_seed']}" / 'model.json',
                        destination / 'dispatch_no_congestion.json')
        card['ablation'] = ablation_report
        card['all_model_sha256'].update({
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(destination.glob('dqn_no_congestion_seed_*.json'))
        })
    write_json(destination / 'model_card.json', card)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenarios', type=Path, default=Path('ml/mission/scenarios'))
    parser.add_argument('--training', type=Path)
    parser.add_argument('--ablation', type=Path)
    parser.add_argument('--models', type=Path,
                        default=Path('ros2_ws/src/capstone_mission/models'))
    args = parser.parse_args()
    export_scenarios(args.scenarios)
    if args.training:
        release_models(args.training, args.models, args.ablation)


if __name__ == '__main__':
    main()
