"""DQN на NumPy: replay buffer, target network, Huber loss и Adam."""

import json

import numpy as np

from spec import FEATURE_VERSION, HIDDEN_SIZE, INPUT_SIZE, OUTPUT_SIZE


class Network:
    """Один скрытый ReLU-слой; тот же порядок весов читает C++ inference."""

    def __init__(self, rng):
        self.parameters = {
            'w1': rng.normal(0.0, np.sqrt(2.0 / INPUT_SIZE),
                             (HIDDEN_SIZE, INPUT_SIZE)).astype(np.float32),
            'b1': np.zeros(HIDDEN_SIZE, dtype=np.float32),
            'w2': rng.normal(0.0, 0.05, (OUTPUT_SIZE, HIDDEN_SIZE)).astype(np.float32),
            'b2': np.zeros(OUTPUT_SIZE, dtype=np.float32),
        }
        self.momentum = {name: np.zeros_like(value) for name, value in self.parameters.items()}
        self.variance = {name: np.zeros_like(value) for name, value in self.parameters.items()}
        self.updates = 0

    def predict(self, inputs):
        values = np.asarray(inputs, dtype=np.float32)
        hidden = np.maximum(values @ self.parameters['w1'].T + self.parameters['b1'], 0.0)
        return hidden @ self.parameters['w2'].T + self.parameters['b2']

    def copy_from(self, source):
        for name in self.parameters:
            self.parameters[name][...] = source.parameters[name]

    def update(self, inputs, actions, targets, learning_rate=0.001):
        hidden = np.maximum(inputs @ self.parameters['w1'].T + self.parameters['b1'], 0.0)
        values = hidden @ self.parameters['w2'].T + self.parameters['b2']
        error = values[np.arange(len(actions)), actions] - targets
        loss = np.where(np.abs(error) <= 1.0, 0.5 * error ** 2, np.abs(error) - 0.5).mean()
        value_gradient = np.zeros_like(values)
        value_gradient[np.arange(len(actions)), actions] = np.clip(error, -1.0, 1.0) / len(actions)
        hidden_gradient = (value_gradient @ self.parameters['w2']) * (hidden > 0.0)
        gradients = {
            'w2': value_gradient.T @ hidden, 'b2': value_gradient.sum(axis=0),
            'w1': hidden_gradient.T @ inputs, 'b1': hidden_gradient.sum(axis=0),
        }
        self.updates += 1
        for name, gradient in gradients.items():
            # Clipping защищает обучение от редкого большого terminal penalty.
            gradient = np.clip(gradient, -5.0, 5.0)
            self.momentum[name] = 0.9 * self.momentum[name] + 0.1 * gradient
            self.variance[name] = 0.999 * self.variance[name] + 0.001 * gradient ** 2
            momentum = self.momentum[name] / (1.0 - 0.9 ** self.updates)
            variance = self.variance[name] / (1.0 - 0.999 ** self.updates)
            self.parameters[name] -= learning_rate * momentum / (np.sqrt(variance) + 1e-8)
        return float(loss)

    def export(self, path, metadata=None):
        document = {
            'schema_version': 1, 'feature_version': FEATURE_VERSION,
            'input_size': INPUT_SIZE, 'hidden_size': HIDDEN_SIZE, 'output_size': OUTPUT_SIZE,
            **{name: value.ravel().tolist() for name, value in self.parameters.items()},
            'metadata': metadata or {},
        }
        path.write_text(json.dumps(document, separators=(',', ':'), allow_nan=False) + '\n')

    @classmethod
    def load(cls, path):
        document = json.loads(path.read_text())
        expected = (INPUT_SIZE, HIDDEN_SIZE, OUTPUT_SIZE)
        shape = (document['input_size'], document['hidden_size'], document['output_size'])
        if document['schema_version'] != 1 or shape != expected:
            raise ValueError('unsupported network schema')
        result = cls(np.random.default_rng(0))
        for name, value in result.parameters.items():
            weights = np.asarray(document[name], dtype=np.float32).reshape(value.shape)
            if not np.isfinite(weights).all():
                raise ValueError('nonfinite model weights')
            value[...] = weights
        return result


class ReplayBuffer:
    """Кольцевой буфер хранит только доступное политике наблюдение и переход."""

    def __init__(self, capacity=20000):
        self.inputs = np.zeros((capacity, INPUT_SIZE), dtype=np.float32)
        self.next_inputs = np.zeros_like(self.inputs)
        self.actions = np.zeros(capacity, dtype=np.int64)
        self.rewards = np.zeros(capacity, dtype=np.float32)
        self.discounts = np.zeros(capacity, dtype=np.float32)
        self.next_masks = np.zeros((capacity, OUTPUT_SIZE), dtype=bool)
        self.capacity = capacity
        self.size = 0
        self.cursor = 0

    def append(self, inputs, action, reward, next_inputs, next_mask, done, duration):
        index = self.cursor
        self.inputs[index] = inputs
        self.actions[index] = action
        self.rewards[index] = reward
        self.next_inputs[index] = next_inputs
        self.next_masks[index] = next_mask
        # Время между решениями неодинаково: discount считается за simulated second.
        self.discounts[index] = 0.0 if done else 0.995 ** duration
        self.cursor = (self.cursor + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def train_batch(self, network, target, rng, batch_size=64):
        indices = rng.integers(0, self.size, size=batch_size)
        next_values = target.predict(self.next_inputs[indices])
        masked_values = np.where(self.next_masks[indices], next_values, -np.inf)
        targets = self.rewards[indices] + self.discounts[indices] * masked_values.max(axis=1)
        return network.update(self.inputs[indices], self.actions[indices], targets)
