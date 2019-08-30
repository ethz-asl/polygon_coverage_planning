#!/usr/bin/python
# -*- coding: utf-8 -*-

# polygon_coverage_planning implements algorithms for coverage planning in
# general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
# Systems Lab, ETH Zürich
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

colors = {'our_bcd': 'r',
          'our_tcd': 'g',
          'one_dir_gk': 'b',
          'gtsp_exact': 'c',
          'one_dir_exact': 'm'}

mmToInch = 0.0393701
width = 117 * mmToInch
aspect = 3

def linFit(df, planner, uniform_weight=False):
    is_planner = df['planner'] == planner
    df_filtered = df[is_planner]
    df_cost = df_filtered[['planner', 'num_hole_vertices', 'cost']]
    x = df_cost['num_hole_vertices'].values
    y = df_cost['cost'].values
    if uniform_weight:
        popt = np.polyfit(x, y, 1)
    else:
        w = np.ones(len(y))
        w[1] = 1000
        popt = np.polyfit(x, y, 1, w=w)
    x = np.arange(x.min(), x.max() + 1)
    yy = popt[0] * x + popt[1]

    return x, yy

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
    our_bcd_fit['x'], our_bcd_fit['y'] = expFit(df, 'our_bcd', uniform_weight=False)
    our_bcd_fit['planner'] = 'our_bcd'

    our_tcd_fit = pd.DataFrame()
    our_tcd_fit['x'], our_tcd_fit['y'] = expFit(df, 'our_tcd', uniform_weight=False)
    our_tcd_fit['planner'] = 'our_tcd'

    one_dir_gk_fit = pd.DataFrame()
    one_dir_gk_fit['x'], one_dir_gk_fit['y'] = expFit(df, 'one_dir_gk', uniform_weight=False)
    one_dir_gk_fit['planner'] = 'one_dir_gk'

    gtsp_exact_fit = pd.DataFrame()
    gtsp_exact_fit['x'], gtsp_exact_fit['y'] = expFit(df, 'gtsp_exact', uniform_weight=True)
    gtsp_exact_fit['planner'] = 'gtsp_exact'

    one_dir_exact_fit = pd.DataFrame()
    one_dir_exact_fit['x'], one_dir_exact_fit['y'] = expFit(df, 'one_dir_exact', uniform_weight=True)
    one_dir_exact_fit['planner'] = 'one_dir_exact'

    return pd.concat([our_bcd_fit, our_tcd_fit, one_dir_gk_fit, gtsp_exact_fit, one_dir_exact_fit])

def createCostFits(df):
    our_bcd_fit = pd.DataFrame()
    our_bcd_fit['x'], our_bcd_fit['y'] = linFit(df, 'our_bcd', uniform_weight=False)
    our_bcd_fit['planner'] = 'our_bcd'

    our_tcd_fit = pd.DataFrame()
    our_tcd_fit['x'], our_tcd_fit['y'] = linFit(df, 'our_tcd', uniform_weight=False)
    our_tcd_fit['planner'] = 'our_tcd'

    one_dir_gk_fit = pd.DataFrame()
    one_dir_gk_fit['x'], one_dir_gk_fit['y'] = linFit(df, 'one_dir_gk', uniform_weight=False)
    one_dir_gk_fit['planner'] = 'one_dir_gk'

    gtsp_exact_fit = pd.DataFrame()
    gtsp_exact_fit['x'], gtsp_exact_fit['y'] = linFit(df, 'gtsp_exact', uniform_weight=False)
    gtsp_exact_fit['planner'] = 'gtsp_exact'

    one_dir_exact_fit = pd.DataFrame()
    one_dir_exact_fit['x'], one_dir_exact_fit['y'] = linFit(df, 'one_dir_exact', uniform_weight=False)
    one_dir_exact_fit['planner'] = 'one_dir_exact'

    return pd.concat([our_bcd_fit, our_tcd_fit, one_dir_gk_fit, gtsp_exact_fit, one_dir_exact_fit])


def plotResults(file):
    # Open CSV.
    df = pd.read_csv(file, sep=",")
    # Plotting.
    plotTimes(df)
    plotCosts(df)
    plotCostDiff(df)
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

    print("Maximum computation times:")
    print("our_bcd: %.3f" %(df_t_total[df['planner'] == 'our_bcd']['t_total'].max()))
    print("our_tcd: %.3f" %(df_t_total[df['planner'] == 'our_tcd']['t_total'].max()))
    print("one_dir_gk: %.3f" %(df_t_total[df['planner'] == 'one_dir_gk']['t_total'].max()))
    print("gtsp_exact: %.3f" %(df_t_total[df['planner'] == 'gtsp_exact']['t_total'].max()))
    print("one_dir_exact: %.3f" %(df_t_total[df['planner'] == 'one_dir_exact']['t_total'].max()))

    instance = '15/0000'
    pd.set_option('display.max_columns', None)
    print("Computation times instance: %s" %(instance))
    print(df[df['instance'] == instance])

def plotCosts(df):
    g = sns.lmplot('num_hole_vertices', 'cost', data=df, hue='planner', fit_reg=False, size=width, aspect=aspect, legend_out=False)
    g.set(xlim=(df['num_hole_vertices'].min() - 10, df['num_hole_vertices'].max() + 10))
    g.set(ylim=(df['cost'].min() - 100, df['cost'].max() + 100))
    g.set(xlabel="Number of Hole Vertices [1]")
    g.set(ylabel="Cost [s]")
    g.ax.legend(loc="upper left")
    g.savefig("./data/cost.pdf", bbox_inches='tight')

    # Linear fit.
    df_fit = createCostFits(df)
    min = df['cost'].min()
    max = df['cost'].max()
    is_smaller = df_fit['y'] < max
    df_fit = df_fit[is_smaller]

    sns.lineplot('x', 'y', data=df_fit, hue='planner', ax=g.axes[0, 0], legend=False)
    g.set(xlim=(df['num_hole_vertices'].min() - 10, df['num_hole_vertices'].max() + 10))
    g.set(ylim=(df['cost'].min() - 10, df['cost'].max() + 10))
    g.set(xlabel="Number of Hole Vertices [1]")
    g.set(ylabel="Cost [s]")
    g.savefig("./data/cost.pdf", bbox_inches='tight')


def plotCostDiff(df):
    # Baseline
    df_our_bcd = df[df['planner'] == 'our_bcd']

    # Comparisons
    df_our_tcd = df[df['planner'] == 'our_tcd']
    df_our_tcd = df_our_tcd.merge(df_our_bcd, on=['instance'], how='inner')
    df_our_tcd_base = df_our_bcd.merge(df_our_tcd, on=['instance'], how='inner')

    df_one_dir_gk = df[df['planner'] == 'one_dir_gk']
    df_one_dir_gk = df_one_dir_gk.merge(df_our_bcd, on=['instance'], how='inner')
    df_one_dir_gk_base = df_our_bcd.merge(df_one_dir_gk, on=['instance'], how='inner')

    df_gtsp_exact = df[df['planner'] == 'gtsp_exact']
    df_gtsp_exact = df_gtsp_exact.merge(df_our_bcd, on=['instance'], how='inner')
    df_gtsp_exact_base = df_our_bcd.merge(df_gtsp_exact, on=['instance'], how='inner')

    df_one_dir_exact = df[df['planner'] == 'one_dir_exact']
    df_one_dir_exact = df_one_dir_exact.merge(df_our_bcd, on=['instance'], how='inner')
    df_one_dir_exact_base = df_our_bcd.merge(df_one_dir_exact, on=['instance'], how='inner')

    # Difference
    diff_our_bcd =  (df_our_bcd['cost'].values - df_our_bcd['cost'].values) / df_our_bcd['cost'].values
    diff_our_tcd =  (df_our_tcd['cost_x'].values - df_our_tcd_base['cost'].values) / df_our_tcd['cost_x'].values
    diff_one_dir_gk =  (df_one_dir_gk['cost_x'].values - df_one_dir_gk_base['cost'].values) / df_one_dir_gk['cost_x'].values
    diff_gtsp_exact =  (df_gtsp_exact['cost_x'].values - df_gtsp_exact_base['cost'].values) / df_gtsp_exact['cost_x'].values
    diff_df_one_dir_exact =  (df_one_dir_exact['cost_x'].values - df_one_dir_exact_base['cost'].values) / df_one_dir_exact['cost_x'].values

    # Create common df.
    df_diff_our_bcd = pd.DataFrame({'planner': df_our_bcd['planner'].values, 'num_hole_vertices': df_our_bcd['num_hole_vertices'], 'diff': diff_our_bcd})
    df_diff_our_tcd = pd.DataFrame({'planner': df_our_tcd['planner_x'].values, 'num_hole_vertices': df_our_tcd['num_hole_vertices_x'].values, 'diff': diff_our_tcd})
    df_diff_one_dir_gk = pd.DataFrame({'planner': df_one_dir_gk['planner_x'].values, 'num_hole_vertices': df_one_dir_gk['num_hole_vertices_x'].values, 'diff': diff_one_dir_gk})
    df_diff_gtsp_exact = pd.DataFrame({'planner': df_gtsp_exact['planner_x'].values, 'num_hole_vertices': df_gtsp_exact['num_hole_vertices_x'].values, 'diff': diff_gtsp_exact})
    df_diff_df_one_dir_exact = pd.DataFrame({'planner': df_one_dir_exact['planner_x'].values, 'num_hole_vertices': df_one_dir_exact['num_hole_vertices_x'].values, 'diff': diff_df_one_dir_exact})

    df_diff = pd.concat([df_diff_our_bcd, df_diff_our_tcd, df_diff_one_dir_gk, df_diff_gtsp_exact, df_diff_df_one_dir_exact])

    g = sns.lmplot('num_hole_vertices', 'diff', data=df_diff, hue='planner', fit_reg=False, size=width, aspect=aspect, legend_out=False, legend=False)
    g.set(xlim=(df_diff['num_hole_vertices'].min() - 10, df_diff['num_hole_vertices'].max() + 10))
    g.set(xlabel="Number of Hole Vertices [1]")
    g.set(ylabel=r'$\Delta c / c$ [1]')
    g.savefig("./data/delta.pdf", bbox_inches='tight')

    print("Maximum relative difference:")
    print("our_bcd: %.3f" %(df_diff[df_diff['planner'] == 'our_bcd']['diff'].max()))
    print("our_tcd: %.3f" %(df_diff[df_diff['planner'] == 'our_tcd']['diff'].max()))
    print("one_dir_gk: %.3f" %(df_diff[df_diff['planner'] == 'one_dir_gk']['diff'].max()))
    print("gtsp_exact: %.3f" %(df_diff[df_diff['planner'] == 'gtsp_exact']['diff'].max()))
    print("one_dir_exact: %.3f" %(df_diff[df_diff['planner'] == 'one_dir_exact']['diff'].max()))
