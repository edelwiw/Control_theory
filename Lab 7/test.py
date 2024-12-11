import math 
import random 

freq_array = [0.1, 0.3, 0.5, 0.7, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5] # Rad/s
amps = [4, 3.5, 3, 2.5, 2, 1.5, 1, 0.5, 0.3, 0.1, 0.05, 0.03, 0.01]
phases = [1.4, 1.3, 1.2, 1.1, 1, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]

for test_num in range(len(freq_array)):
    with open(f"freq_responses/freq_response_{test_num + 1}.txt", "w") as file:
        print(f"Test number: {test_num}")
        omega = freq_array[test_num]
        for i in range(1000):
            timer = i / 100
            file.write(f"{timer} {amps[test_num] * math.sin(omega * timer + phases[test_num])}\n")
        print(f"Test finished")

