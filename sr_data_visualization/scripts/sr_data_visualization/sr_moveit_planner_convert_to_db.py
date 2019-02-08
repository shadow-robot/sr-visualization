#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll, Ioan Sucan, Luis G. Torres
# Author: Shadow Robot Company

import os
import sqlite3
from optparse import OptionParser


def readLogValue(filevar, desired_token_index, expected_tokens):
    start_pos = filevar.tell()
    tokens = filevar.readline().split()
    for token_index in expected_tokens:
        if not tokens[token_index] == expected_tokens[token_index]:
            # undo the read, if we failed to parse.
            filevar.seek(start_pos)
            return None
    return tokens[desired_token_index]


def readOptionalLogValue(filevar, desired_token_index, expected_tokens={}):
    return readLogValue(filevar, desired_token_index, expected_tokens)


def readRequiredLogValue(name, filevar, desired_token_index, expected_tokens={}):
    result = readLogValue(filevar, desired_token_index, expected_tokens)
    if result is None:
        raise Exception("Unable to read " + name)
    return result


def ensurePrefix(line, prefix):
    if not line.startswith(prefix):
        raise Exception("Expected prefix " + prefix + " was not found")
    return line


def readOptionalMultilineValue(filevar):
    start_pos = filevar.tell()
    line = filevar.readline()
    if not line.startswith("<<<|"):
        filevar.seek(start_pos)
        return None
    value = ''
    line = filevar.readline()
    while not line.startswith('|>>>'):
        value = value + line
        line = filevar.readline()
        if line is None:
            raise Exception("Expected token |>>> missing")
    return value


def readRequiredMultilineValue(filevar):
    ensurePrefix(filevar.readline(), "<<<|")
    value = ''
    line = filevar.readline()
    while not line.startswith('|>>>'):
        value = value + line
        line = filevar.readline()
        if line is None:
            raise Exception("Expected token |>>> missing")
    return value


def readBenchmarkLog(dbname, filenames):
    """Parse benchmark log files and store the parsed data in a sqlite3 database."""

    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')

    # create all tables if they don't already exist
    c.executescript("""CREATE TABLE IF NOT EXISTS experiments
        (id INTEGER PRIMARY KEY AUTOINCREMENT, name VARCHAR(512),
        totaltime REAL, timelimit REAL, memorylimit REAL, runcount INTEGER,
        version VARCHAR(128), hostname VARCHAR(1024), cpuinfo TEXT,
        date DATETIME, seed INTEGER, setup TEXT);
        CREATE TABLE IF NOT EXISTS plannerConfigs
        (id INTEGER PRIMARY KEY AUTOINCREMENT,
        name VARCHAR(512) NOT NULL, settings TEXT);
        CREATE TABLE IF NOT EXISTS enums
        (name VARCHAR(512), value INTEGER, description TEXT,
        PRIMARY KEY (name, value));
        CREATE TABLE IF NOT EXISTS runs
        (id INTEGER PRIMARY KEY AUTOINCREMENT, experimentid INTEGER, plannerid INTEGER,
        FOREIGN KEY (experimentid) REFERENCES experiments(id) ON DELETE CASCADE,
        FOREIGN KEY (plannerid) REFERENCES plannerConfigs(id) ON DELETE CASCADE);
        CREATE TABLE IF NOT EXISTS progress
        (runid INTEGER, time REAL, PRIMARY KEY (runid, time),
        FOREIGN KEY (runid) REFERENCES runs(id) ON DELETE CASCADE)""")

    for filename in filenames:
        print('Processing ' + filename)
        logfile = open(filename, 'r')
        start_pos = logfile.tell()
        libname = readOptionalLogValue(logfile, 0, {1: "version"})
        if libname is None:
            libname = "OMPL"
        logfile.seek(start_pos)
        version = readOptionalLogValue(logfile, -1, {1: "version"})
        if version is None:
            # set the version number to make Planner Arena happy
            version = "0.0.0"
        version = ' '.join([libname, version])
        expname = readRequiredLogValue("experiment name", logfile, -1, {0: "Experiment"})
        hostname = readRequiredLogValue("hostname", logfile, -1, {0: "Running"})
        date = ' '.join(ensurePrefix(logfile.readline(), "Starting").split()[2:])
        expsetup = readRequiredMultilineValue(logfile)
        cpuinfo = readOptionalMultilineValue(logfile)
        rseed = int(readRequiredLogValue("random seed", logfile, 0, {-2: "random", -1: "seed"}))
        timelimit = float(readRequiredLogValue("time limit", logfile, 0, {-3: "seconds", -2: "per", -1: "run"}))
        memorylimit = float(readRequiredLogValue("memory limit", logfile, 0, {-3: "MB", -2: "per", -1: "run"}))
        nrrunsOrNone = readOptionalLogValue(logfile, 0, {-3: "runs", -2: "per", -1: "planner"})
        nrruns = -1
        if nrrunsOrNone is not None:
            nrruns = int(nrrunsOrNone)
        totaltime = float(readRequiredLogValue("total time", logfile, 0, {-3: "collect", -2: "the", -1: "data"}))
        numEnums = 0
        numEnumsOrNone = readOptionalLogValue(logfile, 0, {-2: "enum"})
        if numEnumsOrNone is not None:
            numEnums = int(numEnumsOrNone)
        for i in range(numEnums):
            enum = logfile.readline()[:-1].split('|')
            c.execute('SELECT * FROM enums WHERE name IS "%s"' % enum[0])
            if c.fetchone() is None:
                for j in range(len(enum) - 1):
                    c.execute('INSERT INTO enums VALUES (?,?,?)',
                              (enum[0], j, enum[j + 1]))
        c.execute('INSERT INTO experiments VALUES (?,?,?,?,?,?,?,?,?,?,?,?)',
                  (None, expname, totaltime, timelimit, memorylimit, nrruns,
                   version, hostname, cpuinfo, date, rseed, expsetup))
        experimentId = c.lastrowid
        numPlanners = int(readRequiredLogValue("planner count", logfile, 0, {-1: "planners"}))
        for i in range(numPlanners):
            plannerName = logfile.readline()[:-1]
            print('Parsing data for ' + plannerName)

            if ("cartesian" in filename):
                plannerName += "-c"

            # read common data for planner
            numCommon = int(logfile.readline().split()[0])
            settings = ''
            for j in range(numCommon):
                settings = settings + logfile.readline() + ';'

            # find planner id
            c.execute('SELECT id FROM plannerConfigs WHERE (name=? AND settings=?)',
                      (plannerName, settings,))
            p = c.fetchone()
            if p is None:
                c.execute('INSERT INTO plannerConfigs VALUES (?,?,?)',
                          (None, plannerName, settings,))
                plannerId = c.lastrowid
            else:
                plannerId = p[0]

            # get current column names
            c.execute('PRAGMA table_info(runs)')
            columnNames = [col[1] for col in c.fetchall()]

            # read properties and add columns as necessary
            numProperties = int(logfile.readline().split()[0])
            propertyNames = ['experimentid', 'plannerid']
            for j in range(numProperties):
                field = logfile.readline().split()
                propertyType = field[-1]
                propertyName = '_'.join(field[:-1])
                if propertyName not in columnNames:
                    c.execute('ALTER TABLE runs ADD %s %s' % (propertyName, propertyType))
                propertyNames.append(propertyName)
            # read measurements
            insertFmtStr = 'INSERT INTO runs (' + ','.join(propertyNames) + \
                           ') VALUES (' + ','.join('?' * len(propertyNames)) + ')'
            numRuns = int(logfile.readline().split()[0])
            runIds = []
            for j in range(numRuns):
                values = tuple([experimentId, plannerId] +
                               [None if len(x) == 0 or x == 'nan' or x == 'inf' else x
                                for x in logfile.readline().split('; ')[:-1]])
                c.execute(insertFmtStr, values)
                # extract primary key of each run row so we can reference them
                # in the planner progress data table if needed
                runIds.append(c.lastrowid)

            nextLine = logfile.readline().strip()

            # read planner progress data if it's supplied
            if nextLine != '.':
                # get current column names
                c.execute('PRAGMA table_info(progress)')
                columnNames = [col[1] for col in c.fetchall()]

                # read progress properties and add columns as necessary
                numProgressProperties = int(nextLine.split()[0])
                progressPropertyNames = ['runid']
                for i in range(numProgressProperties):
                    field = logfile.readline().split()
                    progressPropertyType = field[-1]
                    progressPropertyName = "_".join(field[:-1])
                    if progressPropertyName not in columnNames:
                        c.execute('ALTER TABLE progress ADD %s %s' %
                                  (progressPropertyName, progressPropertyType))
                    progressPropertyNames.append(progressPropertyName)
                # read progress measurements
                insertFmtStr = 'INSERT INTO progress (' + \
                               ','.join(progressPropertyNames) + ') VALUES (' + \
                               ','.join('?' * len(progressPropertyNames)) + ')'
                numRuns = int(logfile.readline().split()[0])
                for j in range(numRuns):
                    dataSeries = logfile.readline().split(';')[:-1]
                    for dataSample in dataSeries:
                        values = tuple([runIds[j]] +
                                       [None if len(x) == 0 or x == 'nan' or x == 'inf' else x
                                        for x in dataSample.split(',')[:-1]])
                        try:
                            c.execute(insertFmtStr, values)
                        except sqlite3.IntegrityError:
                            print('Ignoring duplicate progress data. '
                                  'Consider increasing ompl::tools::Benchmark::Request::timeBetweenUpdates.')
                            pass

                logfile.readline()
        logfile.close()
    conn.commit()
    c.close()


if __name__ == "__main__":
    usage = """%prog [options] [<benchmark.log> ...]"""
    parser = OptionParser("A script to parse benchmarking results.\n" + usage)
    parser.add_option("-d", "--database", dest="dbname", default="benchmark.db",
                      help="Filename of benchmark database [default: %default]")
    parser.add_option("-l", "--log_folder", dest="log_folder", default=".",
                      help="Option to save all log files in a folder to a sqlite3 database")
    parser.add_option("-o", "--output_folder", dest="output_folder", default=".",
                      help="Folder to place the sqlite3 databases")
    parser.add_option("-s", "--sort", action="store_true", dest="sort", default=False,
                      help="Sort databases per scene")

    (options, args) = parser.parse_args()

    if options.log_folder:
        if (os.path.isfile(options.dbname)):
            os.remove(options.dbname)
        if (os.path.isdir(options.log_folder)):
            if not options.log_folder.endswith("/"):
                options.log_folder = options.log_folder + "/"
            if not options.output_folder.endswith("/"):
                options.output_folder = options.output_folder + "/"
            files = [f for f in os.listdir(options.log_folder) if os.path.isfile(options.log_folder + f)]

            # Removing previous dbs that already existed
            for i, f in enumerate(files):
                if f.endswith(".log"):
                    if options.sort:
                        scene_name = f.split("_Pose")[0]
                        dbname = options.output_folder + scene_name + "_" + options.dbname
                        if os.path.isfile(dbname):
                            print "Removing db:", dbname
                            os.remove(dbname)
                    else:
                        dbname = options.output_folder + options.dbname
                        if os.path.isfile(dbname):
                            print "Removing db:", dbname
                            os.remove(dbname)

            for i, f in enumerate(files):
                if f.endswith(".log"):
                    if options.sort:
                        # Separate log files per scene
                        scene_name = f.split("_Pose")[0]
                        dbname = options.output_folder + scene_name + "_" + options.dbname
                        print "Sorting files for scene_name", scene_name
                        readBenchmarkLog(dbname, [options.log_folder + f])
                    else:
                        # Add all log files to a single db file
                        dbname = options.output_folder + options.dbname
                        readBenchmarkLog(dbname, [options.log_folder + f])
            print "Databases have been generated in:", options.output_folder
        else:
            parser.error("Bad argument provided for folder with log files. Please provide full path of log file")
