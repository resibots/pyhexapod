import numpy as np
import math


class Controller:
    def __init__(self, params, array_dim=100):
        self.array_dim = array_dim
        self.trajs = self._compute_trajs(params, array_dim)

    def step(self, t):
        k = int(math.floor(t * self.array_dim)) % self.array_dim
        return self.trajs[:, k]

    def _compute_trajs(self, p, array_dim):
        trajs = np.zeros((6 * 3, array_dim))
        k = 0
        for i in range(0, 36, 6):
            trajs[k,:] =  0.5 * self._control_signal(p[i], p[i + 1], p[i + 2], array_dim)
            trajs[k+1,:] = self._control_signal(p[i + 3], p[i + 4], p[i + 5], array_dim)
            trajs[k+2,:] = trajs[k+1,:] 
            k += 3
        return trajs * math.pi / 4.0
        
    def _control_signal(self, amplitude, phase, duty_cycle, array_dim=100):
        '''
        create a smooth periodic function with amplitude, phase, and duty cycle, 
        amplitude, phase and duty cycle are in [0, 1]
        '''
        assert(amplitude >= 0 and amplitude <= 1)
        assert(phase >= 0 and phase <= 1)
        assert(duty_cycle >= 0 and duty_cycle <= 1)
        command = np.zeros(array_dim)

        # create a 'top-hat function'
        up_time = array_dim * duty_cycle
        temp = [amplitude if i < up_time else -amplitude for i in range(0, array_dim)]
        
        # smoothing kernel
        kernel_size = int(array_dim / 10)
        kernel = np.zeros(int(2 * kernel_size + 1))
        sigma = kernel_size / 3
        for i in range(0, len(kernel)):
            kernel[i] =  math.exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma**2)) / (sigma * math.sqrt(math.pi))
        sum = np.sum(kernel)

        # smooth the function
        for i in range(0, array_dim):
            command[i] = 0
            for d in range(1, kernel_size + 1):
                if i - d < 0:
                    command[i] += temp[array_dim + i - d] * kernel[kernel_size - d]
                else:
                    command[i] += temp[i - d] * kernel[kernel_size - d]
            command[i] += temp[i] * kernel[kernel_size]
            for d in range(1, kernel_size + 1):
                if i + d >= array_dim:
                    command[i] += temp[i + d - array_dim] * kernel[kernel_size + d]
                else:
                    command[i] += temp[i + d] * kernel[kernel_size + d]
            command[i] /= sum

        # shift according to the phase
        final_command = np.zeros(array_dim)
        start = math.floor(array_dim * phase)
        current = 0
        for i in range(start, array_dim):
            final_command[current] = command[i]
            current += 1
        for i in range(0, start):
            final_command[current] = command[i]
            current += 1
            
        assert(len(final_command) == array_dim)
        return final_command
