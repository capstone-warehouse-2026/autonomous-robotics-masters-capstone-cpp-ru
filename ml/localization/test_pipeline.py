"""Synthetic unit fixtures only. These are NOT capstone experiment results."""
import os
from pathlib import Path
import tempfile
import unittest
import numpy as np
import pipeline as p


class PipelineTest(unittest.TestCase):
    def test_split_leakage(self):
        a = dict(world='a', trajectory='a1', split='train')
        b = dict(world='a', trajectory='b1', split='test')
        with self.assertRaises(ValueError): p.validate_splits([(None, a), (None, b)])
        b['world'] = 'b'; p.validate_splits([(None, a), (None, b)])

    def test_training_and_export(self):
        rng = np.random.default_rng(9)
        x = rng.normal(size=(60, 7)); y = np.full((60, 3), .002)
        params, info = p.fit(x[:40], y[:40], x[40:], y[40:], epochs=80)
        v, _, _, _ = p.forward(x, params)
        self.assertTrue(np.all(np.isfinite(v)))
        self.assertTrue(np.all((v >= p.LOW) & (v <= p.HIGH)))
        self.assertTrue(np.isfinite(info['validation_nll']))
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'model.txt';p.export_model(path, params)
            values = path.read_text().split();self.assertEqual(len(values), 92)
            raw = np.array(list(map(float, values[1:])))
            h = raw[:64].reshape(8, 8);out = raw[64:].reshape(3, 9)
            reconstructed = [h[:, :7].T, h[:, 7], out[:, :8].T, out[:, 8]]
            np.testing.assert_allclose(p.forward(x, reconstructed)[0], v)

    def test_ablation(self):
        rng = np.random.default_rng(3);x = rng.normal(size=(40, 7));y = np.full((40, 3), .003)
        params, _ = p.fit(x[:25], y[:25], x[25:], y[25:], epochs=10, no_history=True)
        np.testing.assert_equal(params[0][6], 0)

    def test_truth_wrap_and_gap(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)
            (path/'ground_truth.csv').write_text('stamp,x,y,yaw\n0,0,0,3.13\n0.1,1,0,-3.13\n1,2,0,0\n')
            poses, valid = p.truth_at(path, np.array([.05, .5]))
            self.assertAlmostEqual(abs(poses[0, 2]), np.pi)
            np.testing.assert_equal(valid, [True, False])

    def test_metrics_known_error(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d); t = np.arange(31)*.1
            (path/'ground_truth.csv').write_text('stamp,x,y,yaw\n'+'\n'.join(f'{v:.17g},{v:.17g},0,0' for v in t)+'\n')
            names = ['stamp','x','y','yaw']+[f'p{i}{j}' for i in range(3) for j in range(3)]+['mx','nis','accepted','fallback','latency_us']
            rows = np.zeros(len(t), dtype=[('kind','U1')]+[(n, float) for n in names]);rows['kind']='W';rows['stamp']=t;rows['x']=t+.1
            for i in range(3): rows[f'p{i}{i}']=.01
            result = p.metrics((path, {}), rows)
            self.assertAlmostEqual(result['ate_rmse_m'], .1)
            self.assertAlmostEqual(result['nees_mean'], 1.)
            self.assertLess(result['rpe_1s_translation_rmse_m'], 1e-12)
            rows['x']=t+.6; result=p.metrics((path, {}), rows)
            self.assertEqual(result['loss_episodes'],1)
            self.assertAlmostEqual(result['loss_fraction'],1.)
            self.assertTrue(result['unrecovered_at_end'])

    @unittest.skipUnless(os.environ.get('G1_REPLAY'), 'Set G1_REPLAY to test C++ inference parity')
    def test_cpp_replay_inference_parity(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d);cells = np.zeros((20, 30), dtype=int)
            cells[[0, -1], :] = 100;cells[:, [0, -1]] = 100
            (root/'map.txt').write_text('30 20 .1 0 0 0\n'+' '.join(map(str, cells.ravel())))
            angle = np.arange(360)*2*np.pi/360;c, s = np.cos(angle), np.sin(angle)
            radius = np.minimum(1.4/np.maximum(abs(c), 1e-10), .9/np.maximum(abs(s), 1e-10))
            xy = np.column_stack([radius*c, radius*s])
            (root/'events.txt').write_text('W 0 0 0 0 0\nS 0 360 '+' '.join(map(str, xy.ravel()))+'\n')
            rng = np.random.default_rng(4)
            params = [rng.normal(0, .001, (7, 8)), np.zeros(8), rng.normal(0, .1, (8, 3)), np.full(3, -5.)]
            p.export_model(root/'model.txt', params)
            rows = p.replay(Path(os.environ['G1_REPLAY']), (root, {'initial':[1.54, 1.03, .02]}), root/'out.csv', 'learned', root/'model.txt')
            scan = rows[rows['kind'] == 'S'];self.assertEqual(len(scan), 1)
            self.assertEqual(scan['fallback'][0], 0)
            x = np.column_stack([scan[f'f{i}'] for i in range(7)])
            expected = p.forward(x, params)[0]
            actual = np.column_stack([scan[k] for k in ('rx', 'ry', 'ryaw')])
            np.testing.assert_allclose(actual, expected, rtol=1e-6)
            self.assertEqual(scan['accepted'][0], 1)

if __name__ == '__main__': unittest.main()
