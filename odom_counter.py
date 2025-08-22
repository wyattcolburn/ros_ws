#!/usr/bin/env python3
"""
ROS2 Bag Odometry Counter Utility

This script scans a directory containing ROS2 bag files and counts
the total number of odometry messages across all bags.
"""

import os
import subprocess
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple
import argparse


def run_ros2_bag_info(bag_path: str) -> str:
    """
    Run 'ros2 bag info' command on a bag directory and return the output.

    Args:
        bag_path: Path to the ROS2 bag directory

    Returns:
        Command output as string, or empty string if command fails
    """
    try:
        result = subprocess.run(
            ['ros2', 'bag', 'info', bag_path],
            capture_output=True,
            text=True,
            check=True
        )
        return result.stdout
    except subprocess.CalledProcessError as e:
        print(f"Error running ros2 bag info on {bag_path}: {e}")
        return ""
    except FileNotFoundError:
        print("Error: 'ros2' command not found. Make sure ROS2 is installed and sourced.")
        sys.exit(1)


def parse_bag_info(info_output: str) -> Dict[str, int]:
    """
    Parse the output of 'ros2 bag info' to extract topic names and message counts.
    Uses simple string operations instead of complex regex.

    Args:
        info_output: Output string from 'ros2 bag info' command

    Returns:
        Dictionary mapping topic names to message counts
    """
    topics = {}

    # Split into lines
    lines = info_output.split('\n')
    in_topics_section = False

    for line in lines:
        # Check for start of topics section - but also process this line if it has topic data
        if 'Topic information:' in line:
            in_topics_section = True
            # DON'T continue here - process this line too in case it has topic data

        # If we're in topics section and line has both Topic and Count
        if in_topics_section and 'Topic:' in line and 'Count:' in line:
            try:
                # Find positions of Topic: and Count:
                topic_pos = line.find('Topic:')
                count_pos = line.find('Count:')

                if topic_pos != -1 and count_pos != -1:
                    # Extract topic name (everything after Topic: until next |)
                    topic_start = topic_pos + 6  # Skip "Topic:"
                    topic_section = line[topic_start:]

                    if '|' in topic_section:
                        topic_name = topic_section.split('|')[0].strip()
                    else:
                        # Fallback: take first word
                        topic_name = topic_section.split()[0].strip()

                    # Extract count (everything after Count: until next |)
                    count_start = count_pos + 6  # Skip "Count:"
                    count_section = line[count_start:]

                    if '|' in count_section:
                        count_str = count_section.split('|')[0].strip()
                    else:
                        # Fallback: take first word
                        count_str = count_section.split()[0].strip()

                    # Convert to int and store
                    if topic_name and count_str.isdigit():
                        topics[topic_name] = int(count_str)

            except (ValueError, IndexError, AttributeError):
                # Skip malformed lines
                continue

        # Exit topics section on empty line or non-topic content
        elif in_topics_section and line.strip() == '':
            continue
        elif in_topics_section and line.strip() and 'Topic:' not in line and 'Topic information:' not in line:
            # Reached end of topics section
            break

    return topics


def is_odometry_topic(topic_name: str) -> bool:
    """
    Determine if a topic name is likely an odometry topic.

    Args:
        topic_name: ROS2 topic name

    Returns:
        True if the topic appears to be odometry-related
    """
    odom_keywords = [
        'odom', 'odometry', 'pose', 'position',
        '/odom', '/odometry', '/pose', '/robot_pose'
    ]

    topic_lower = topic_name.lower()
    return any(keyword in topic_lower for keyword in odom_keywords)


def scan_bag_directory(directory: str, verbose: bool = False, debug: bool = False) -> Tuple[int, Dict[str, int]]:
    """
    Scan a directory for ROS2 bags and count odometry messages.

    Args:
        directory: Path to directory containing ROS2 bag directories
        verbose: If True, print detailed information about each bag
        debug: If True, print raw ros2 bag info output for debugging

    Returns:
        Tuple of (total_odom_count, bag_details)
    """
    if not os.path.exists(directory):
        print(f"Error: Directory {directory} does not exist")
        sys.exit(1)

    total_odom_count = 0
    bag_details = {}

    # Get all subdirectories (potential bag directories)
    subdirs = [d for d in os.listdir(directory)
               if os.path.isdir(os.path.join(directory, d))]

    if not subdirs:
        print(f"No subdirectories found in {directory}")
        return 0, {}

    print(f"Scanning {len(subdirs)} directories for ROS2 bags...")

    for subdir in sorted(subdirs):
        bag_path = os.path.join(directory, subdir)

        if verbose:
            print(f"\n--- Processing {subdir} ---")

        # Get bag info
        info_output = run_ros2_bag_info(bag_path)
        if not info_output:
            if verbose:
                print(
                    f"  Skipping {subdir} (not a valid ROS2 bag or error occurred)")
            continue

        # Debug: show raw output
        if debug:
            print(f"  Raw ros2 bag info output:")
            print("  " + "\n  ".join(info_output.split('\n')))
            print()

        # Parse topics and counts
        topics = parse_bag_info(info_output)

        if debug:
            print(f"  Parsed topics: {topics}")
            print()

        if not topics:
            if verbose:
                print(f"  No topics found in {subdir}")
            continue

        # Count odometry messages
        bag_odom_count = 0
        odom_topics = []

        for topic, count in topics.items():
            if is_odometry_topic(topic):
                bag_odom_count += count
                odom_topics.append((topic, count))

        if bag_odom_count > 0:
            total_odom_count += bag_odom_count
            bag_details[subdir] = bag_odom_count

            if verbose:
                print(f"  Odometry topics found:")
                for topic, count in odom_topics:
                    print(f"    {topic}: {count} messages")
                print(f"  Total odometry messages: {bag_odom_count}")
        else:
            if verbose:
                print(f"  No odometry topics found")
                print(f"  Available topics: {list(topics.keys())}")

    return total_odom_count, bag_details


def main():
    parser = argparse.ArgumentParser(
        description='Count odometry messages in ROS2 bag files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 odom_counter.py ~/ros_ws/ros_bag
  python3 odom_counter.py . --verbose
  python3 odom_counter.py . --debug
  python3 odom_counter.py /path/to/bags --list-topics
        """
    )

    parser.add_argument(
        'directory',
        nargs='?',
        default='.',
        help='Directory containing ROS2 bag subdirectories (default: current directory)'
    )

    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Print detailed information about each bag'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='Print raw ros2 bag info output for debugging parsing issues'
    )

    parser.add_argument(
        '--list-topics',
        action='store_true',
        help='List all topics found in bags (useful for debugging topic detection)'
    )

    args = parser.parse_args()

    # Convert to absolute path
    directory = os.path.abspath(args.directory)

    print(f"Scanning directory: {directory}")

    if args.list_topics:
        # Special mode to list all topics for debugging
        subdirs = [d for d in os.listdir(directory)
                   if os.path.isdir(os.path.join(directory, d))]

        all_topics = set()
        for subdir in subdirs:
            bag_path = os.path.join(directory, subdir)
            info_output = run_ros2_bag_info(bag_path)
            if info_output:
                topics = parse_bag_info(info_output)
                all_topics.update(topics.keys())

        print("\nAll topics found across all bags:")
        for topic in sorted(all_topics):
            odom_marker = " [ODOMETRY]" if is_odometry_topic(topic) else ""
            print(f"  {topic}{odom_marker}")
        return

    # Main counting logic
    total_count, bag_details = scan_bag_directory(
        directory, args.verbose, args.debug)

    print(f"\n{'='*50}")
    print("SUMMARY")
    print(f"{'='*50}")

    if bag_details:
        print(f"Bags with odometry data: {len(bag_details)}")
        if not args.verbose:
            print("\nOdometry count per bag:")
            for bag_name, count in sorted(bag_details.items()):
                print(f"  {bag_name}: {count:,} messages")

        print(f"\nTOTAL ODOMETRY MESSAGES: {total_count:,}")
    else:
        print("No odometry messages found in any bags.")
        print("\nTip: Use --list-topics to see all available topics")
        print("     and verify odometry topic detection is working correctly.")
        print("     Use --debug to see raw ros2 bag info output.")


if __name__ == "__main__":
    main()
