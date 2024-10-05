import os
import argparse


def process_file(input_filepath, output_filepath):
    # Read the file
    with open(input_filepath, 'r') as file:
        lines = file.readlines()

    processed_lines = []
    line_count = len(lines)

    # Process lines based on the given rules
    for i, line in enumerate(lines):
        if i == line_count - 1:
            last_line = line
            continue

        # Find the part that matches "Ahae/XXXX/YY-nbx ZZZZ;"
        parts = line.split("/")
        if len(parts) < 3:
            continue

        # Determine the replacement value for XXXX
        if i < 16 * 12:  # First 192 lines
            ahae_id = 13 - (i // 16)
            parts[1] = f"{ahae_id:01}"
        elif i == 192:  # Special case for line 193
            parts[1] = "Interval"
        else:  # After line 193, XXXX = 1
            parts[1] = "1"

        # Reconstruct the line
        processed_line = "/".join(parts)
        processed_lines.append(processed_line)

    # Write the processed lines to the output file
    with open(output_filepath, 'w') as file:
        file.writelines(processed_lines)
        file.write(last_line)  # Append the unprocessed last line


def process_directory(input_directory, output_directory):
    # Ensure the output directory exists
    os.makedirs(output_directory, exist_ok=True)

    # Iterate through all files in the input directory
    for filename in os.listdir(input_directory):
        input_filepath = os.path.join(input_directory, filename)
        output_filepath = os.path.join(output_directory, filename)

        if os.path.isfile(input_filepath):
            process_file(input_filepath, output_filepath)


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Suseup")
    parser.add_argument('input_directory', type=str,
                        help="Path to the input directory")
    parser.add_argument('output_directory', type=str,
                        help="Path to the output directory")

    args = parser.parse_args()

    # Call the processing function with the provided directories
    process_directory(args.input_directory, args.output_directory)


if __name__ == "__main__":
    main()
