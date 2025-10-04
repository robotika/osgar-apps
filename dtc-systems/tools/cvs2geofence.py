import csv


def csv2geofence(filename):
    with open(filename) as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            print(f"              [{row['Latitude']}, {row['Longitude']}],")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Convert DARPA course csv to OSGAR geofence')
    parser.add_argument('filename', help='Input CSV file')
    args = parser.parse_args()

    csv2geofence(args.filename)
