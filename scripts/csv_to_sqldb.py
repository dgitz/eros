#!/usr/bin/python
from __future__ import print_function
import sqlite3
import csv
import os
import glob
import sys
def print_usage():
    print("Usage Instructions")
    print("Convert csv directory to sql database: python csv_to_sqldb -c <CSV Directory> <Database Path>.")

def convert(csv_directory,database_path):
    db = database_path
    if(os.path.exists(db)):
        os.remove(db)
    conn = sqlite3.connect(db)
    conn.text_factory = str  # allows utf-8 data to be stored
     
    c = conn.cursor()
     
    # traverse the directory and process each .csv file
    for csvfile in glob.glob(os.path.join(csv_directory, "*.csv")):
        # remove the path and extension and use what's left as a table name
        tablename = os.path.splitext(os.path.basename(csvfile))[0]
        if((tablename == 'FASTDatabaseTable_ProjectMedia') or
           (tablename == 'FASTDatabaseSwitchboard Items')):
            skip = 1
        else:
            skip = 0
        if(skip == 1):
            print("[WARN] Unsupported Table: " + tablename + " Skipping.")
        else:
            print("Processing Table: " + tablename)
            with open(csvfile, "rb") as f:
                reader = csv.reader(f)
         
                header = True
                for row in reader:
                    if header:
                        # gather column names from the first row of the csv
                        header = False
         
                        #sql = "DROP TABLE IF EXISTS %s" % tablename
                        #c.execute(sql)
                        sql = "CREATE TABLE %s (%s)" % (tablename,
                                  ", ".join([ "%s text" % column for column in row ]))
                        #print(sql)
                        c.execute(sql)
         
                        for column in row:
                            if column.lower().endswith("_id"):
                                index = "%s__%s" % ( tablename, column )
                                sql = "CREATE INDEX %s on %s (%s)" % ( index, tablename, column )
                                c.execute(sql)
         
                        insertsql = "INSERT INTO %s VALUES (%s)" % (tablename,
                                    ", ".join([ "?" for column in row ]))
         
                        rowlen = len(row)
                    else:
                        # skip lines that don't have the right number of columns
                        if len(row) == rowlen:
                            c.execute(insertsql, row)
         
                conn.commit()
     
    c.close()
    conn.close()

if len(sys.argv) == 1:
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-c"):
    convert(sys.argv[2],sys.argv[3])
