import numpy as np
import pickle
import matplotlib.pyplot as plt


def load_data(f):
    with open(f, "rb") as f:
        return pickle.load(f)
