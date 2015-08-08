import inspect
import settings
import utils
import ipdb

from plan_refinement import PlanRefinement
# from interface.env_manager import EnvManager


class PRCreator(object):
    def __init__(self, env):
        self.env = env

    def create_pr(self, file_object_or_name, parent_pr, resume_from):
        from box_world import BoxWorld
        self.world = BoxWorld(self.env)
        self.pr = PlanRefinement(self.env, self.world)

        if type(file_object_or_name) is str:
            file_obj = open(file_object_or_name)
        else:
            file_obj = file_object_or_name
        functions = self._parse(file_obj)

        for lineno, instruction in enumerate(functions):
            action_name = instruction["name"] + "(" + ", ".join(instruction['args']) + ")"
            if lineno < resume_from:
                print "Adding Static: {}".format(action_name)
                # import pdb; pdb.set_trace()
                self.pr.add_action(parent_pr.action_list[lineno].make_static_copy())
            else:
                print "Adding: {}".format(action_name)
                # instruction['method'](*(lineno,) + tuple(instruction['args']))
                self.pr.add_action(*(lineno, instruction['method']) + tuple(instruction['args']))

        return self.pr

    def _parse(self, file_obj):
        functions = []
        for l in file_obj:
            function = {}
            stripped_line = l.strip(" \t\n")

            if stripped_line.startswith('#'):
                continue
            if len(stripped_line) < 2:
                continue

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
            method = getattr(self.world, fn_name, None)

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

    def _add_action_move(self, lineno, start, end):
        # print "added move with ", start, " ", end
        self.pr.add_move(lineno, start, end)
        return

    def _add_action_move_w_obj(self, lineno, start, end, obj, gp):
        # print "added move_w_obj ", start, " ", end, " ", obj, " ", gp
        self.pr.add_move(lineno, start, end, obj, gp)
        return
    
    def _add_action_pick(self, lineno, pos, obj, loc, gp):
        # print "added pick ", pos, " ", obj, " ", loc, " ", gp
        self.pr.add_pick(lineno, pos, obj, loc, gp)
        return

    def _add_action_place(self, lineno, pos, obj, loc, gp):
        # print "added place ", pos, " ", obj, " ", loc, " ", gp
        self.pr.add_place(lineno, pos, obj, loc, gp)
        return

