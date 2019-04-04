import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

colors = {'our_bcd': 'r',
          'our_tcd': 'g',
          'one_dir_gkma': 'b',
          'gtsp_exact': 'c',
          'one_dir_exact': 'm'}

def plotResults(file):
    # Open CSV.
    df = pd.read_csv(file, sep=",")
    # Plotting.
    plotTimes(df)

def plotTimes(df):
    t = df[['timer_setup_total','timer_solve_total']].values
    t_total = np.sum(t, axis=1)
    df_t_total = pd.DataFrame(t_total, columns=['t_total'])
    df_planner = df[['planner', 'num_hole_vertices']]
    df_t_total = df_planner.join(df_t_total)

    sns.lmplot('num_hole_vertices', 't_total', data=df_t_total, hue='planner', fit_reg=False)
    plt.show()
