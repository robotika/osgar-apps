import pandas as pd
import matplotlib.pyplot as plt
import argparse
import sys

def plot_coordinates(csv_file, extra_coords=None):
    """
    Plots coordinates from a CSV file and optional extra coordinates.

    Args:
        csv_file (str): Path to the CSV file.
        extra_coords (list of tuples): A list of (lat, lon) tuples to plot as extra points.
    """
    try:
        # Read the data from the CSV file
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: The file '{csv_file}' was not found.")
        sys.exit(1)
    
    # Check if required columns exist
    required_cols = ['casualty_id', 'lat', 'lon']
    if not all(col in df.columns for col in required_cols):
        print(f"Error: CSV file must contain 'casualty_id', 'lat', and 'lon' columns.")
        sys.exit(1)

    # Create a plot
    plt.figure(figsize=(12, 10))
    
    # Plot the data from the CSV file
    plt.scatter(df['lon'], df['lat'], alpha=0.7, label='Casualties (from CSV)')

    # Add casualty_id labels to each point
    for index, row in df.iterrows():
        plt.text(row['lon'], row['lat'], str(row['casualty_id']), fontsize=9, ha='right')

    # Plot extra coordinates if they were passed
    if extra_coords:
        extra_lons = [lon for lat, lon in extra_coords]
        extra_lats = [lat for lat, lon in extra_coords]
        plt.scatter(extra_lons, extra_lats, color='red', marker='X', s=150, label='Extra Points (CLI)')
        # Label the extra points
        for i, (lat, lon) in enumerate(extra_coords):
            plt.text(lon, lat, f'Extra {i+1}', fontsize=10, color='red', ha='left')

    # Customize and show the plot
    plt.title('Geographical Plot of Casualties')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')  # Ensure aspect ratio is equal for geographical data
    plt.show()

if __name__ == "__main__":
    # Set up the argument parser to handle command-line arguments
    parser = argparse.ArgumentParser(
        description="Plot geographical coordinates from a CSV file and add extra points."
    )
    
    parser.add_argument(
        '--file',
        type=str,
        required=True,
        help="Path to the input CSV file."
    )
    
    parser.add_argument(
        '--extra-coords',
        type=float,
        nargs='*',  # Allows for zero or more coordinate values
        help="Extra coordinates to plot, provided as: lat1 lon1 lat2 lon2 ..."
    )
    
    args = parser.parse_args()
    
    # Process the extra coordinates from a flat list into pairs of (lat, lon)
    extra_points = []
    if args.extra_coords:
        if len(args.extra_coords) % 2 != 0:
            print("Error: Extra coordinates must be provided in pairs (latitude longitude).")
            sys.exit(1)
        # Create an iterator from the list
        it = iter(args.extra_coords)
        # Zip the iterator with itself to create pairs
        extra_points = list(zip(it, it))
        
    # Call the main plotting function
    plot_coordinates(args.file, extra_points)
