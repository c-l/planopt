import inspect
import settings
import utils
import ipdb
import re

from plan_refinement import PlanRefinement
from IPython import embed as shell
# from interface.env_manager import EnvManager

import settings


class PRCreator(object):
    def __init__(self, env):
        self.env = env
        self.putdown_mode = "PDP"

    def create_pr(self, file_object_or_name, parent_pr, resume_from):
        self.pddlToOpt = settings.pddlToOpt(self.env)
        self.pr = PlanRefinement(self.env, self.pddlToOpt)

        if type(file_object_or_name) is str:
            file_obj = open(file_object_or_name)
        else:
            file_obj = file_object_or_name
        functions = self._parse(file_obj)

        for lineno, instruction in enumerate(functions):
            action_name = instruction["name"] + "(" + ", ".join(instruction['args']) + ")"
            if lineno < resume_from:
                raise Exception("Should never happen because resumeFrom = 0!")
                # print "Adding Static: {}".format(action_name)
                # self.pr.add_action(parent_pr.action_list[lineno].make_static_copy())
                # self.pr.add_parent_action(lineno, parent_pr.action_list[lineno])
                self.pr.add_action(*(lineno, instruction['method']) + tuple(instruction['args']))
            else:
                print "Adding: {}".format(action_name)
                # instruction['method'](*(lineno,) + tuple(instruction['args']))
                self.pr.add_action(*(lineno, instruction['method']) + tuple(instruction['args']))

        return self.pr

    def _uniqueify_symbols_in_line(self, stripped_line):
        stripped_line += " " # to make replacing easier
        matches = re.findall("GP_([A-Z0-9]*)", stripped_line)
        if matches:
            for obj in matches:
                curr_count = self.pr.gp_counters.setdefault(obj, 0)
                stripped_line = stripped_line.replace("GP_%s "%(obj), "GP_%s_%d "%(obj, curr_count))
        matches = re.findall("GRASP_([A-Z0-9]*)", stripped_line)
        if matches:
            for obj in matches:
                curr_count = self.pr.grasp_counters.setdefault(obj, 0)
                stripped_line = stripped_line.replace("GRASP_%s "%(obj), "GRASP_%s_%d "%(obj, curr_count))
        matches = re.findall("%s_(\w*)"%self.putdown_mode, stripped_line)
        if matches:
            for obj in matches:
                curr_count = self.pr.pdp_counters.setdefault(obj, 0)
                stripped_line = stripped_line.replace("%s_%s "%(self.putdown_mode, obj), "%s_%s_%d "%(self.putdown_mode, obj, curr_count))
        stripped_line = stripped_line[:-1]

        match = re.search("PICK \w* \w* GP_([A-Z0-9]*)", stripped_line)
        if match:
            gp = match.groups()[0]
            self.pr.gp_counters[gp] = self.pr.gp_counters.setdefault(gp, 0) + 1
        match = re.search("PLACE \w* \w* PDP_([A-Z0-9]*_[A-Z0-9]*)\w* GRASP_([A-Z0-9]*)", stripped_line)
        if match:
            pdp, grasp = match.groups()
            self.pr.pdp_counters[pdp] = self.pr.pdp_counters.setdefault(pdp, 0) + 1
            self.pr.grasp_counters[grasp] = self.pr.grasp_counters.setdefault(grasp, 0) + 1

        # moveto start pose correction
        match = re.search("MOVE\w* (\w+_\d+) \w+", stripped_line)
        if match:
            pose = match.groups()[0]
            correction = max(int(pose.split("_")[-1]) - 1, 0)
            new_pose = pose[:pose.rfind("_")] + "_" + str(correction)
            stripped_line = stripped_line.replace(pose, new_pose)

        return stripped_line

    def _parse(self, file_obj):
        functions = []
        for l in file_obj:
            function = {}
            stripped_line = l.strip(" \t\n")

            if stripped_line.startswith('#'):
                continue
            if len(stripped_line) < 2:
                continue

            stripped_line = self._uniqueify_symbols_in_line(stripped_line)

            tokens_list = stripped_line.split(" ")[1:]  # skip instruction number
            parsed_fn_name = tokens_list[0].lower()
            # if 'move' in parsed_fn_name:
            #     splitted = parsed_fn_name.split("_")
            #     if len(splitted) == 4:
            #       parsed_fn_name = "_".join([splitted[0], splitted[3]])
            #     else:
            #       parsed_fn_name = "moveto"

            # fn_name = "_add_action_" + parsed_fn_name
            # method = getattr(self, fn_name, None)
            fn_name = parsed_fn_name
            method = getattr(self.pddlToOpt, fn_name, None)

            if method is None:
                raise TypeError("No method called %s" % (parsed_fn_name))

            function['name'] = parsed_fn_name
            function['method'] = method
            function['args'] = []
            num_args = len(inspect.getargspec(method).args[1:])  # remove self
            default_args = inspect.getargspec(method).defaults
            if default_args is not None:
                num_args -= len(default_args)  # remove default args
            num_variables = len(tokens_list[1:])  # remove function name
            # num_variables += 1  # also passing in lineno
            num_variables += 4  # also passing in lineno, pr, env, robot
            if num_args > num_variables:
                raise TypeError("Wrong arguments for %s, expected %d, got %d" % (
                    fn_name, num_args, num_variables))

            args = (t.strip("\n")
                    for t in tokens_list[1:] if len(t) > 0)
            for arg in args:
                #grasping location ignored!
                if arg.startswith("gp_"):
                    pass
                function['args'].append(arg.lower())

            functions.append(function)
        return functions
