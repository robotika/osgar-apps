#!/usr/bin/env python3
"""
BEV Road Mapping - Consecutive Frame Pair Alignment Stats Plotter.
Processes all consecutive pairs in a dataset, exports statistics to a CSV,
and generates a high-quality visualization plot of 'Pixels Within Tolerance (%)'
alongside other metrics like Raw Overlap (%) and Grayscale RMSE.
"""

import argparse
import csv
import sys
from pathlib import Path

# Use Agg backend for matplotlib to prevent GUI popups
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

from bevroad.check_pair import evaluate_pair
from bevroad.utils import parse_csv


def main():
    parser = argparse.ArgumentParser(
        description='Processes all consecutive pairs in a BEV road mapping dataset and plots alignment stats.'
    )
    parser.add_argument('csv_path', help='Path to overview.csv metadata file')
    parser.add_argument(
        '--config', default=None, help='Path to calibration bev.json file (defaults to matching JSON in CSV folder)'
    )
    parser.add_argument(
        '--road-width', type=float, default=2.0, help='Physical width of the road in meters (default: 2.0)'
    )
    parser.add_argument(
        '--lane-width-fraction',
        type=float,
        default=0.5,
        help='Fraction of the BEV width that the road width occupies (default: 0.5)',
    )
    parser.add_argument(
        '--near',
        type=float,
        default=1.0,
        help='Forward distance of the BEV image bottom edge from the robot center, in meters (default: 1.0)',
    )
    parser.add_argument(
        '--resolution',
        type=float,
        default=0.02,
        help='Resolution of the mosaic image in meters per pixel (default: 0.02)',
    )
    parser.add_argument(
        '--tolerance',
        type=float,
        default=15.0,
        help='Grayscale tolerance threshold (0-255) for pixel alignment comparison (default: 15.0)',
    )
    parser.add_argument(
        '--output', default=None, help='Path to save the plotted image (default: <imdir>/alignment_plot.png)'
    )
    parser.add_argument(
        '--csv-out', default=None, help='Path to save the statistics CSV (default: <imdir>/alignment_stats.csv)'
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    imdir = csv_path.parent

    # Determine default output file paths if not specified
    plot_path = Path(args.output) if args.output else imdir / 'alignment_plot.png'
    csv_out_path = Path(args.csv_out) if args.csv_out else imdir / 'alignment_stats.csv'

    print(f'Loading metadata CSV: {csv_path}')
    records = parse_csv(csv_path)
    total_records = len(records)

    if total_records < 3:
        print(f'Error: Dataset must contain at least 3 frames to evaluate N+2 coverage. Found {total_records} frames.')
        sys.exit(1)

    max_valid_index = total_records - 3
    print(f'Found {total_records} records. Processing indices 0 to {max_valid_index}...')

    indices = []
    pct_within_tols = []
    overlap_pct_raws = []
    pixel_rmses = []
    pixel_maes = []

    # Process each pair
    for index in range(max_valid_index + 1):
        print(f'\rProcessing pair {index}/{max_valid_index}...', end='', flush=True)
        try:
            # Call evaluate_pair in headless mode (no_vis=True)
            _, metrics = evaluate_pair(
                csv_path=str(csv_path),
                index=index,
                config=args.config,
                road_width=args.road_width,
                lane_width_fraction=args.lane_width_fraction,
                near=args.near,
                resolution=args.resolution,
                tolerance=args.tolerance,
                no_vis=True,
                save_debug=False,
                verbose=False,
            )
            indices.append(index)
            pct_within_tols.append(metrics['pct_within_tol'])
            overlap_pct_raws.append(metrics['overlap_pct_raw'])
            pixel_rmses.append(metrics['pixel_rmse'])
            pixel_maes.append(metrics['pixel_mae'])
        except Exception as e:
            print(f'\nError processing index {index}: {e}')
            sys.exit(1)

    print('\nProcessing complete! Saving results...')

    # Write metrics to CSV
    try:
        with open(csv_out_path, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    'Index',
                    'Remaining_Overlap_Within_Tolerance_Pct',
                    'Raw_Overlap_Pct',
                    'Grayscale_RMSE',
                    'Grayscale_MAE',
                ]
            )
            for i in range(len(indices)):
                writer.writerow([indices[i], pct_within_tols[i], overlap_pct_raws[i], pixel_rmses[i], pixel_maes[i]])
        print(f'Saved alignment statistics CSV to: {csv_out_path}')
    except Exception as e:
        print(f'Error writing CSV file: {e}')

    # Generate the visualization plot
    try:
        fig, ax1 = plt.subplots(figsize=(10, 6))

        # First axis: Percentages (Within Tolerance and Raw Overlap)
        color = 'tab:blue'
        ax1.set_xlabel('Frame Pair Index (N to N+1)')
        ax1.set_ylabel('Percentage (%)', color=color)

        # Plot within tolerance line
        line1 = ax1.plot(
            indices,
            pct_within_tols,
            color='dodgerblue',
            marker='o',
            linewidth=2,
            label=f'In-Tol Pixels % (tol={args.tolerance})',
        )
        # Plot raw overlap line
        line2 = ax1.plot(
            indices, overlap_pct_raws, color='darkgray', linestyle='--', linewidth=1.5, label='Raw Overlap %'
        )
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.set_ylim(-5, 105)
        ax1.grid(True, linestyle=':', alpha=0.6)

        # Second axis: RMSE (for quality trend)
        ax2 = ax1.twinx()
        color = 'tab:red'
        ax2.set_ylabel('Grayscale RMSE', color=color)
        line3 = ax2.plot(
            indices, pixel_rmses, color='crimson', marker='x', linestyle=':', linewidth=1.5, label='Grayscale RMSE'
        )
        ax2.tick_params(axis='y', labelcolor=color)
        # Auto scale or fixed scale for RMSE based on max value
        if len(pixel_rmses) > 0 and max(pixel_rmses) > 0:
            ax2.set_ylim(0, max(pixel_rmses) * 1.2)
        else:
            ax2.set_ylim(0, 50)

        # Title and legends
        plt.title('Consecutive BEV Frame Pair Alignment Quality Metrics', fontsize=14, fontweight='bold', pad=15)

        # Merge legends from both axes
        lines = line1 + line2 + line3
        labels = [ln.get_label() for ln in lines]
        ax1.legend(lines, labels, loc='lower left', framealpha=0.9)

        plt.tight_layout()
        plt.savefig(plot_path, dpi=300)
        plt.close()

        print(f'Generated quality metrics plot: {plot_path}')
    except Exception as e:
        print(f'Error generating plot: {e}')


if __name__ == '__main__':
    main()
