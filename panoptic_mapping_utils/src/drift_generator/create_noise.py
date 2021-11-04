import numpy as np


def main():
    num_random_numbers = 10000 * 6
    mean = 0.00001
    std = 0.0001
    file_name = "noise.txt"
    random_numbers = np.random.randn(num_random_numbers) * std + mean
    random_numbers_str = ','.join([str(n) for n in list(random_numbers)])
    with open(file_name, "w") as fw:
        fw.write(random_numbers_str)
        fw.write(',')

if __name__ == "__main__":
    main()