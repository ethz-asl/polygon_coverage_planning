import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

colors = {'our_bcd': 'r',
          'our_tcd': 'g',
          'one_dir_gkma': 'b',
          'gtsp_exact': 'c',
          'one_dir_exact': 'm'}

mmToInch = 0.0393701
width = 117 * mmToInch
aspect = 3

def expFit(df, planner, uniform_weight=False):
    is_planner = df['planner'] == planner
    df_filtered = df[is_planner]
    df_times = df_filtered[['planner', 'num_hole_vertices', 't_total']]
    x = df_times['num_hole_vertices'].values
    y = df_times['t_total'].values
    if uniform_weight:
        popt = np.polyfit(x, np.log(y), 1)
    else:
        popt = np.polyfit(x, np.log(y), 1, w=np.sqrt(y))
    a = np.exp(popt[1])
    b = popt[0]

    x = np.arange(x.min(), x.max() + 1)
    yy = a * np.exp(b * x)

    return x, yy

def createFits(df):
    our_bcd_fit = pd.DataFrame()
    our_bcd_fit['x'], our_bcd_fit['y'] = expFit(df, 'our_bcd')
    our_bcd_fit['planner'] = 'our_bcd'

    our_tcd_fit = pd.DataFrame()
    our_tcd_fit['x'], our_tcd_fit['y'] = expFit(df, 'our_tcd')
    our_tcd_fit['planner'] = 'our_tcd'

    one_dir_gkma_fit = pd.DataFrame()
    one_dir_gkma_fit['x'], one_dir_gkma_fit['y'] = expFit(df, 'one_dir_gkma')
    one_dir_gkma_fit['planner'] = 'one_dir_gkma'

    gtsp_exact_fit = pd.DataFrame()
    gtsp_exact_fit['x'], gtsp_exact_fit['y'] = expFit(df, 'gtsp_exact', uniform_weight=True)
    gtsp_exact_fit['planner'] = 'gtsp_exact'

    one_dir_exact_fit = pd.DataFrame()
    one_dir_exact_fit['x'], one_dir_exact_fit['y'] = expFit(df, 'one_dir_exact', uniform_weight=False)
    one_dir_exact_fit['planner'] = 'one_dir_exact'

    return pd.concat([our_bcd_fit, our_tcd_fit, one_dir_gkma_fit, gtsp_exact_fit, one_dir_exact_fit])

def plotResults(file):
    # Open CSV.
    df = pd.read_csv(file, sep=",")
    # Plotting.
    plotTimes(df)
    plotCosts(df)
    plt.show()

def plotTimes(df):
    # Plot times
    t = df[['timer_setup_total','timer_solve_total']].values
    t_total = np.sum(t, axis=1)
    df_t_total = pd.DataFrame(t_total, columns=['t_total'])
    df_planner = df[['planner', 'num_hole_vertices']]
    df_t_total = df_planner.join(df_t_total)

    # Scatter plot
    g = sns.lmplot('num_hole_vertices', 't_total', data=df_t_total, hue='planner', fit_reg=False, legend=False, legend_out=False, size=width, aspect=aspect)

    # Exponential fit.
    df_fit = createFits(df_t_total)
    min = df_t_total['t_total'].min()
    max = df_t_total['t_total'].max()
    is_smaller = df_fit['y'] < max
    df_fit = df_fit[is_smaller]

    sns.lineplot('x', 'y', data=df_fit, hue='planner', ax=g.axes[0, 0], legend=False)
    g.set(xlim=(df_t_total['num_hole_vertices'].min() - 10, df_t_total['num_hole_vertices'].max() + 10))
    g.set(ylim=(df_t_total['t_total'].min() - 10, df_t_total['t_total'].max() + 10))
    g.set(xlabel="Number of Hole Vertices [1]")
    g.set(ylabel="Runtime [s]")
    g.savefig("./data/time.pdf", bbox_inches='tight')

def plotCosts(df):
    g = sns.lmplot('num_hole_vertices', 'cost', data=df, hue='planner', fit_reg=False, size=width, aspect=aspect, legend_out=False)
    g.set(xlim=(df['num_hole_vertices'].min() - 10, df['num_hole_vertices'].max() + 10))
    g.set(ylim=(df['cost'].min() - 100, df['cost'].max() + 100))
    g.set(xlabel="Number of Hole Vertices [1]")
    g.set(ylabel="Cost [s]")
    g.ax.legend(loc="upper left")
    g.savefig("./data/cost.pdf", bbox_inches='tight')
