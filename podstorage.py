import numpy as np
import pandas as pd
import random

# a_sr, s rows, r cols

def random_warehouse_distribution(s, r, n, distribution_type=random):
    """
    Generates a matrix representing the distribution of SKUs in storage pods.

    Parameters:
    s (int): Number of SKUs in the warehouse.
    r (int): Number of storage pods in the warehouse.
    n (int): Number of each SKU stored in the warehouse.

    Returns:
    A matrix with s rows and r columns, where each cell represents
    how many of each SKU is stored in each pod, distributed randomly.
    """
    # Initialize the matrix with zeros
    distribution_matrix = np.zeros((s, r), dtype=int)
    
    # Distribute each SKU to the pods randomly
    for sku in range(s):
        pods = random.choices(range(r), k=n)
        for pod in pods:
            distribution_matrix[sku, pod] += 1
    
    return distribution_matrix


def flexible_warehouse_distribution(s, r, n, k):
    """
    Generates a matrix representing the distribution of SKUs in storage pods,
    where each SKU is stored in up to k different pods.

    Parameters:
    s (int): Number of SKUs in the warehouse.
    r (int): Number of storage pods in the warehouse.
    n (int): Number of each SKU stored in the warehouse.
    k (int): Maximum number of pods in which each SKU can be stored.

    Returns:
    A matrix with s rows and r columns, where each cell represents
    how many of each SKU is stored in each pod, with each SKU stored in up to k pods.
    """
    # Initialize the matrix with zeros
    distribution_matrix = np.zeros((s, r), dtype=int)
    
    # Distribute each SKU to up to k pods
    for sku in range(s):
        num_pods = random.randint(1, min(k, r))  # Ensure we don't exceed the total number of pods
        pods = random.sample(range(r), num_pods)  # Select unique pods
        for pod in pods:
            # Distribute n items of the SKU evenly across the selected pods
            distribution_matrix[sku, pod] = n // num_pods
        # If there's a remainder, distribute it randomly among the selected pods
        remainder = n % num_pods
        for i in range(remainder):
            distribution_matrix[sku, pods[i]] += 1
    
    return distribution_matrix

def flexible_amount_warehouse_distribution(s, r, k, lower_bound, upper_bound):
    """
    Generates a matrix representing the distribution of SKUs in storage pods,
    where each SKU is stored in up to k different pods. The amount of each SKU
    is determined by a random selection within the interval [lower_bound, upper_bound].

    Parameters:
    s (int): Number of SKUs in the warehouse.
    r (int): Number of storage pods in the warehouse.
    k (int): Maximum number of pods in which each SKU can be stored.
    lower_bound (int): Lower bound of the interval for the amount of each SKU.
    upper_bound (int): Upper bound of the interval for the amount of each SKU.

    Returns:
    A matrix with s rows and r columns, where each cell represents
    how many of each SKU is stored in each pod, with each SKU stored in up to k pods.
    """
    # Initialize the matrix with zeros
    distribution_matrix = np.zeros((s, r), dtype=int)
    
    # Distribute each SKU to up to k pods with a random amount in the specified interval
    for sku in range(s):
        n = random.randint(lower_bound, upper_bound)  # Random amount for this SKU
        num_pods = random.randint(1, min(k, r))  # Ensure we don't exceed the total number of pods or k
        pods = random.sample(range(r), num_pods)  # Select unique pods
        for pod in pods:
            # Distribute n items of the SKU evenly across the selected pods
            distribution_matrix[sku, pod] = n // num_pods
        # If there's a remainder, distribute it randomly among the selected pods
        remainder = n % num_pods
        for i in range(remainder):
            distribution_matrix[sku, pods[i]] += 1
    
    return distribution_matrix

if __name__ == "__main__":
    s = 5  # Number of SKUs
    r = 10  # Number of storage pods
    n = 20  # Number of each SKU stored in the warehouse
    k = 5  # Maximum number of pods for each SKU
    lower_bound = 10  # Lower bound of the amount interval
    upper_bound = 20  # Upper bound of the amount interval

    # random_warehouse = random_warehouse_distribution(s, r, n)
    # print(random_warehouse)

    # distribution_matrix = flexible_warehouse_distribution(s, r, n, k)
    # print(distribution_matrix)

    distribution_matrix = flexible_amount_warehouse_distribution(s, r, k, lower_bound, upper_bound)
    print(distribution_matrix)
