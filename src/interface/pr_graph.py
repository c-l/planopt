import copy
import settings
import networkx as nx
import pylab
import pprint

from interface.env_manager import EnvManager
# from hl_actions.hl_action import ActionError, InstantiationExhaustedException
from pr_creator import PRCreator

pp = pprint.PrettyPrinter()

class PRGraph(object):
    def __init__(self, env):
        # import pdb; pdb.set_trace()
        self.env = env
        self.plan_refinements = {}
        self.saved_environments = {}
        self.saved_plans = {}
        self.saved_random_states = {}
        self.pr_creator = PRCreator(self.env)
        self.graph = nx.DiGraph()
        self.starting_state = EnvManager.save_openrave_state(self.env)


    def resume(self, plan_key):
        # try:
        print "Resuming plan:"
        self.saved_plans[plan_key].printPlan()

        if plan_key in self.saved_environments:
            EnvManager.restore_openrave_state(self.env, self.saved_environments[plan_key])

        settings.RANDOM_STATE = copy.deepcopy(self.saved_random_states[plan_key])

        self.plan_refinements[plan_key].get_next_instantiation()

        import ipdb; ipdb.set_trace() # BREAKPOINT
        self.plan_refinements[plan_key].setActionListNames(self.saved_plans[plan_key])

        # except ActionError, e:
        #     self.saved_environments[plan_key] = EnvManager.save_openrave_state(self.env)
        #     self.saved_random_states[plan_key] = copy.deepcopy(settings.RANDOM_STATE)
        #     e.cur_plan = self.saved_plans[plan_key]
        #     self._handle_error(e)
        #     raise e

        cur_plan = self.plan_refinements[plan_key]
        # print "Backtracking took: {} seconds".format(cur_plan.timings['backtrack'])
        # print "Motion planning took: {} seconds".format(cur_plan.timings['mp'])

        # print "Replanning with margins..."
        # cur_plan.replan_with_margins(self.starting_state)

        if not settings.run_test_mode[0]:
            cur_plan.execute_all(self.starting_state)
        else:
            try:
                cur_plan.execute_all_silent(self.starting_state)
            except ActionError, e:
                self.saved_environments[plan_key] = EnvManager.save_openrave_state(self.env)
                self.saved_random_states[plan_key] = copy.deepcopy(settings.RANDOM_STATE)
                e.cur_plan = self.saved_plans[plan_key]
                self._handle_error(e)
                import ipdb; ipdb.set_trace()
        print "Done"


    def addEdge(self, plan_key, new_plan_key, generated_plan, resume_from, error_str):
        # import pdb; pdb.set_trace()
        parent_pr = None
        if plan_key in self.plan_refinements:
            self.plan_refinements[plan_key].restore_openrave_state(resume_from)
            parent_pr = self.plan_refinements[plan_key]

        plan_fname = generated_plan.getStrIOPlan()
        pr = self.pr_creator.create_pr(plan_fname, parent_pr, resume_from)
        pr.set_resume_from(resume_from)
        self.plan_refinements[new_plan_key] = pr
        self.saved_plans[new_plan_key] = generated_plan
        self.saved_random_states[new_plan_key] = copy.deepcopy(settings.RANDOM_STATE)
        self.update_graph(plan_key, new_plan_key, error_str)


    def is_duplicate_plan(self, key):
        node_list = self.graph.nodes()
        node_list.remove("_")
        for node in node_list:
            if self.saved_plans[node].equals(self.saved_plans[key]):
                return True
        return False


    def error_explored(self, parent_key, label):
        edgeLabels = map(lambda x: self.graph[x[0]][x[1]]['label'], self.graph.out_edges(parent_key))
        if label in edgeLabels:
            return True


    def update_graph(self, parent_key, child_key, label):
        # import pdb; pdb.set_trace()
        nodeList = self.graph.nodes()
        if len(nodeList) == 0:
            self.graph_source = parent_key
            self.graph.add_node(parent_key)
        elif parent_key not in nodeList:
            raise Exception("Parent key {} not in graph!".format(parent_key))
        if (not self.is_duplicate_plan(child_key)) and (not self.error_explored(parent_key, label)):
            print "\nAdded edge {} --> {}\n".format(parent_key, child_key)
            self.graph.add_node(child_key)
            self.graph.add_edge(parent_key, child_key, label=label)
        else:
            print "\nDuplicate edge/node skipped.\n"


    def show_graph(self):
#         layout = nx.spring_layout(self.graph)
#         pylab.figure(1)
#         nx.draw(self.graph, layout)
#         nx.draw_networkx_edge_labels(self.graph, layout, font_size=2)
#         pylab.show()
#        A = nx.to_agraph(self.graph)
#        A.layout()
#        A.draw("test.png")

        print "Current PRGraph:"
        for edge in self.graph.edges(data=True):
            print "{} --> {}: \n{}\n".format(edge[0], edge[1], edge[2]['label'])

    def _handle_error(self, error):
        assert isinstance(error, ActionError)

        print "Handling an error"
