import os
import json
from jinja2 import Environment, FileSystemLoader


def get_reports_data(reports_path, images_path):
    """
    Reads JSON reports and maps them to corresponding images.

    Args:
        reports_path (str): Path to the folder with JSON reports.
        images_path (str): Path to the folder with images.

    Returns:
        list: A list of dictionaries, where each dictionary is a report.
    """
    reports = []
    for filename in sorted(os.listdir(reports_path)):
        if filename.endswith(".json"):
            report_id = os.path.splitext(filename)[0].replace('report', '')
            filepath = os.path.join(reports_path, filename)
            with open(filepath, 'r') as f:
                report_data = json.load(f)
                report_data['id'] = report_id

                # Find a matching image (jpg, png, etc.)
                image_found = None
                for img_ext in ['.jpg', '.jpeg', '.png', '.gif']:
                    potential_image = f"image{report_id}{img_ext}"
                    if os.path.exists(os.path.join(images_path, potential_image)):
                        image_found = os.path.join(images_path, potential_image)
                        break
                report_data['image'] = image_found
                reports.append(report_data)
    return reports


def main():
    """ Main function to generate the webpage """
    # Setup paths
    reports_path = 'reports'
    images_path = 'images'
    templates_path = 'templates'

    # Load the Jinja2 template
    env = Environment(loader=FileSystemLoader(templates_path))
    template = env.get_template('template.html')

    # Get the data
    reports_data = get_reports_data(reports_path, images_path)

    # Render the HTML
    output_html = template.render(reports=reports_data)

    # Write the output file
    with open('output.html', 'w') as f:
        f.write(output_html)
    print("Successfully generated output.html")


if __name__ == "__main__":
    main()
