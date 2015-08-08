# To change this template, choose Tools | Templates
# and open the template in the editor.
import networkx as nx
import pdb
import sys
#    could also add two forms of each new plan: one for exhaustive search and one for returning errors.

class GraphTraversal:
    def __init__(self, prgraph, source):
        self.prgraph = prgraph
        self.source = source
        
 
    def getSearchRoutine(self, searchAlgo):
        return getattr(self, searchAlgo)()
   

    def daisyChainBFS(self):    
        currentGraph = self.prgraph.graph.copy()
        while True: 
            oldGraph = currentGraph
            for edge in nx.bfs_edges(currentGraph, self.source):
                yield edge[1]   
            currentGraph = self.prgraph.graph.copy()
            if currentGraph.nodes() == oldGraph.nodes():
                raise StopIteration

         
    def default(self):
        while True:
            nodes = self.prgraph.graph.nodes()
            nodes.sort(key=len)
            
            selected = nodes[-1]
            if selected == '_':
                selected = '*'
                
            yield selected

        
    def IDIBDFS(self):
        if self.source == '_':
            node = '*'
        else:
            node = self.source
            
        for bound in range(1, 4):
            print "bound "  + repr(bound)
            
            stack = [node]
            addCount = {}
            while len(stack)>0:
                current = stack.pop()
                if current != "_":
                    yield current   
                if self.getDepth(node, current) < bound:
                    self.addKChildren(current, bound, stack, addCount)
    #            print "2"
                print stack


    def getDepth(self, source, target):
        return len(nx.shortest_path(self.prgraph.graph, source, target))-1
            
    
    def addKChildren(self, node, k, stack, addCount): 
        children =  self.prgraph.graph.successors(node) 
        children.sort()
        delta = len(children) - k
        for child in children:
            addCount[child] = addCount.setdefault(child, 0) + 1
            if addCount[child] > k:
                children.remove(child)
                
        if delta >= 0:
            print "adding {} children".format(k)
            stack.extend(children[:k])
        else:
            print "adding {} children".format(len(children))
            stack.extend(children)
            print "adding {} copies".format(-1*delta)
            for i in range(-1*delta):
                if addCount.setdefault(node, 0) <= k:
                    stack.append(node)
                    addCount[node] += 1
#    print "1"
#    print stack