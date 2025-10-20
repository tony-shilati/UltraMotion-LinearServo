import pickle
import matplotlib
filename = "fft_plot_20251020_163926.fig.pkl"
path = "../outputs/" + filename
with open(path, 'rb') as f:
    fig = pickle.load(f)
fig.show()