#!/usr/bin/env python

import optparse, logging, sys, os
from datetime import datetime
import time
import common

logger = logging.getLogger("GECCO2011")

### MAIN SETTINGS ###

# name - domain - instance
SAMPLES = [("IPC6_SEQ_ELEVATORS_12",
            "/tools/pddl/ipc/ipc2008/seq-sat/elevators-strips/p12-domain.pddl",
            "/tools/pddl/ipc/ipc2008/seq-sat/elevators-strips/p12.pddl"),

           ("IPC6_TEMPO_OPENSTACKS_17",
            "/tools/pddl/ipc/ipc2008/tempo-sat/openstacks-strips/p17-domain.pddl",
            "/tools/pddl/ipc/ipc2008/tempo-sat/openstacks-strips/p17.pddl"),

           ("IPC3_STRIPS_DEPOTS_13",
            "/tools/pddl/ipc/IPC3/Tests1/Depots/Strips/Depots.pddl",
            "/tools/pddl/ipc/IPC3/Tests1/Depots/Strips/pfile13"),

           ("IPC3_STRIPS_DRIVERLOG_11",
            "/tools/pddl/ipc/IPC3/Tests1/DriverLog/Strips/driverlog.pddl",
            "/tools/pddl/ipc/IPC3/Tests1/DriverLog/Strips/pfile11"),

           ("IPC6_COST_SCANALYSER_22",
            "/tools/pddl/ipc/ipc2008/seq-sat/scanalyzer-strips/p22-domain.pddl",
            "/tools/pddl/ipc/ipc2008/seq-sat/scanalyzer-strips/p22.pddl"),

           ("IPC6_TEMPO_PARCPRINTER_11",
            "/tools/pddl/ipc/ipc2008/tempo-sat/parcprinter-strips/p11-domain.pddl",
            "/tools/pddl/ipc/ipc2008/tempo-sat/parcprinter-strips/p11.pddl"),
           ]

### MAIN SETTINGS ENDS ###

PATTERN_TIME_FILENAME = "%(TIMEDIR)s/%(NAME)s_%(FIELD)s.time.%(NUM)s"
PATTERN_RES_FILENAME = "%(RESDIR)s/%(NAME)s_%(FIELD)s.out.%(NUM)s"
PATTERN_PLAN_FILENAME = "%(RESDIR)s/%(NAME)s_%(FIELD)s.soln.%(NUM)s"

PATTERN_CMD = \
    "/usr/bin/time -v -o %(TIME_FILENAME)s "\
    "timelimit -t 1800 "\
    "./%(COMMAND)s "\
    "--domain=%(DOMAIN)s "\
    "--instance=%(INSTANCE)s "\
    "--plan-file=%(PLAN_FILENAME)s "\
    "--runs-max=%(RUNMAX)s "\
    "--popSize=%(POPSIZE)s "\
    "--offsprings=%(OFFSPRINGS)s "\
    "--max-seconds=%(MAXSECONDS)s "\
    "--gen-steady=%(GENSTEADY)s "\
    "--parallelize-loop=%(LOOP)s "\
    "--parallelize-nthreads=%(THREADS)s "\
    "--parallelize-dynamic=%(DYNAMIC)s "\
    "> %(RES_FILENAME)s"

parser = optparse.OptionParser()
parser.add_option('-e', '--execute', action='store_true', default=False, help='execute experiences')
parser.add_option('-D', '--dynamic', action='store_true', default=False, help='use the dynamic scheduler in openmp')
parser.add_option('-p', '--plot', action='store_true', default=False, help='plot data')
parser.add_option('-N', '--nruns', type='int', default=11, help='give here a number of runs')

topic = str(datetime.today())
for char in [' ', ':', '-', '.']: topic = topic.replace(char, '_')
parser.add_option('-t', '--topic', default=topic, help='give here a topic name used to create the folder')

parser.add_option('-C', '--cores', action='store_true', default=False, help='enable results on processus variations')
parser.add_option('-S', '--sizes', action='store_true', default=False, help='enable results on population sizes variations')

options = common.parser(parser)

if options.plot:
    import pylab

def do_proc(resdir, timedir):
    """
    EXPS on IPC6_SEQ_ELEVATORS_12 & IPC6_TEMPO_OPENSTACKS_17
    steadyState=50
    1) popsize=48 & runmax=1 & maxseconds=0
    2) RESTART case: popsize=96 & runmax=0 & maxseconds=1799
    foreach nthreads: 1, 24, 48
    repeat 11 times
    """

    if not options.cores: return

    for field, popsize, runmax, maxseconds in [
        ("PROC", 48, 1, 0),
        #("RESTART_PROC", 96, 0, 1799)
        ]:
        for name, domain, instance in SAMPLES:
            local_logger = logging.getLogger("GECCO2011.PROC.%s" % name)
            plotdata = []
            for num in range(1, options.nruns+1):
                subdata = []
                for nthreads in [1, 24, 48]:
                    field_name = "%s_%s_%d" % (field, "DYNAMIC" if options.dynamic else "STATIC", nthreads)
                    time_filename = PATTERN_TIME_FILENAME % {"TIMEDIR": timedir, "NAME": name, "FIELD": field_name, "NUM": num}
                    res_filename = PATTERN_RES_FILENAME % {"RESDIR": resdir, "NAME": name, "FIELD": field_name, "NUM": num}
                    plan_filename = PATTERN_PLAN_FILENAME % {"RESDIR": resdir, "NAME": name, "FIELD": field_name, "NUM": num}
                    cmd = PATTERN_CMD % {"DOMAIN": domain,
                                         "INSTANCE": instance,
                                         "LOOP": 1,
                                         "DYNAMIC": 1 if options.dynamic else 0,
                                         "THREADS": nthreads,
                                         "RUNMAX": runmax,
                                         "POPSIZE": popsize,
                                         "OFFSPRINGS": popsize*7,
                                         "MAXSECONDS": maxseconds,
                                         "GENSTEADY": 50,
                                         "TIME_FILENAME": time_filename,
                                         "RES_FILENAME": res_filename,
                                         "PLAN_FILENAME": plan_filename,
                                         }
                    local_logger.debug(cmd)
                    if options.execute:
                        os.system( cmd )
                    if options.plot:
                        try:
                            f = open(time_filename).readlines()
                            t1 = float(f[1].split()[-1])
                            tp = f[4].split()[-1].split(':')
                            tp = float(int(tp[0]) * 60 + float(tp[1]))
                            subdata.append([t1, tp, t1 / tp])
                        except IOError:
                            pass

                if options.plot:
                    if len(subdata):
                        plotdata.append(subdata)

            if options.plot:
                local_logger.info(plotdata)
                pylab.boxplot( plotdata )

def do_pop(resdir, timedir):
    """
    EXPS on IPC6_SEQ_ELEVATORS_12 & IPC6_TEMPO_OPENSTACKS_17
    nthreads=48 & steadyState=50
    1) runmax=1 & maxseconds=0
    2) RESTART case: runmax=0 & maxseconds=0
    foreach popSize: 1, 24, 48, 72, 96
    repeat 11 times
    """

    if not options.sizes: return

    for field, runmax, maxseconds, command in [
        #("POP", 1, 0),
        ("RESTART_POP", 0, 0, "dae"),
        ("RESTART_POP_SHARED_MEMOIZATION", 0, 0, "dae_shared_memoization")
        ]:
        for name, domain, instance in SAMPLES:
            local_logger = logging.getLogger("GECCO2011.POP.%s" % name)
            plotdata = []
            for num in range(1, options.nruns+1):
                subdata = []
                #for popsize in [1, 24, 48, 72, 96]:
                for popsize in [48, 96, 1152, 2304, 3456, 4608]:
                    field_name = "%s_%s_%d" % (field, "DYNAMIC" if options.dynamic else "STATIC", popsize)
                    time_filename = PATTERN_TIME_FILENAME % {"TIMEDIR": timedir, "NAME": name, "FIELD": field_name, "NUM": num}
                    res_filename = PATTERN_RES_FILENAME % {"RESDIR": resdir, "NAME": name, "FIELD": field_name, "NUM": num}
                    plan_filename = PATTERN_PLAN_FILENAME % {"RESDIR": resdir, "NAME": name, "FIELD": field_name, "NUM": num}
                    cmd = PATTERN_CMD % {"COMMAND": command,
                                         "DOMAIN": domain,
                                         "INSTANCE": instance,
                                         "LOOP": 1,
                                         "DYNAMIC": 1 if options.dynamic else 0,
                                         "THREADS": 48,
                                         "RUNMAX": runmax,
                                         "POPSIZE": popsize,
                                         "OFFSPRINGS": popsize*7,
                                         "MAXSECONDS": maxseconds,
                                         "GENSTEADY": 50,
                                         "TIME_FILENAME": time_filename,
                                         "RES_FILENAME": res_filename,
                                         "PLAN_FILENAME": plan_filename,
                                         }
                    local_logger.debug(cmd)
                    if options.execute:
                        os.system( cmd )
                    if options.plot:
                        try:
                            f = open(time_filename).readlines()
                            t1 = float(f[1].split()[-1])
                            tp = f[4].split()[-1].split(':')
                            tp = float(int(tp[0]) * 60 + float(tp[1]))
                            #subdata.append([t1, tp, t1 / tp])
                            subdata.append(t1 / tp)
                        except IOError:
                            pass

                if options.plot:
                    if len(subdata):
                        plotdata.append(subdata)

            if options.plot:
                local_logger.info( plotdata )

                newdata = []
                for i in range(0,5):
                    newdata.append([0] * 9)

                for i in range(0,9):
                    for j in range(0,5):
                        newdata[j][i] = plotdata[i][j]

                print "NEWDATA"
                print newdata

                pylab.boxplot( newdata )
                pylab.xlabel('population sizes')
                pylab.ylabel('speedup with 48 cores')
                pylab.savefig( name + '_output.pdf', format='pdf' )
                pylab.savefig( name + '_output.png', format='png' )
                pylab.cla()
                pylab.clf()

def main():
    resdir = "%s/Res" % options.topic
    timedir = "%s/Time" % options.topic

    # create needed directories
    if options.execute:
        for d in [resdir, timedir]:
            try:
                os.makedirs(d)
            except OSError:
                pass

    do_proc(resdir, timedir)
    do_pop(resdir, timedir)

# when executed, just run main():
if __name__ == '__main__':
    logger.debug('### script started ###')
    main()
    logger.debug('### script ended ###')
