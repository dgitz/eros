# Bag File Conversion
To convert a Bag file (or directory of Bag Files) to CSV's, do the following:<br>
`roscd eros`<br>
`python scripts/process_logs.py -d <BAG FILE DIRECTORY> -o <CSV FILE OUTPUT DIRECTORY>`


# CSV File Analysis
To Analyze a directory of CSV Files, do the following:<br>
`roscd eros`<br>
`python scripts/analyze_logs.py -d <CSV FILE DIRECTORY> -o <ANALYSIS FILE OUTPUT DIRECTORY>`