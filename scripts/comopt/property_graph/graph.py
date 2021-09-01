#
#    ComOpT: COMbination and OPtimization for Testing autonomous driving systems
#
#    Copyright (C) 2021  Yuhang Chen, Chih-Hong Cheng, Changwen Li, Tiantian Sun, Rongjie Yan
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Affero General Public License as published
#    by the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Affero General Public License for more details.
#
#    You should have received a copy of the GNU Affero General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

from comopt.property_graph.property import for_printing
from loguru import logger as log

error_descriptions = {
    'DUPLICATE_VERTEX_ID': 'Duplicate vertex id',
    'DUPLICATE_EDGE_ID': 'Duplicate edge id',
}


class GraphError(RuntimeError):
    def __init__(self, code):
        self.code = code
        self.description = error_descriptions[code]


class Vertex:
    def __init__(self, _id, properties):
        self.id = _id
        self.properties = properties
        self.in_edges = list()
        self.out_edges = list()
        self.deleted = False

    def __str__(self):
        delete_string = ''
        if self.deleted:
            delete_string = ',DELETED'
        return f'Vertex[{self.id}{delete_string}]' \
               f'{{properties={for_printing(self.properties)},{self.in_edges}->V->{self.out_edges}}}'


class Edge:
    def __init__(self, _id, from_id, to_id, properties):
        self.id = _id
        self.properties = properties
        self.from_id = from_id
        self.to_id = to_id
        self.deleted = False

    def __str__(self):
        delete_string = ''
        if self.deleted:
            delete_string = ',DELETED'
        return f'Edge[{self.id}{delete_string}]({self.from_id}->{self.to_id})' \
               f'{{properties={for_printing(self.properties)}}} '

    def cmp(self, other):
        return self.id == other.id and self.properties == other.properties \
               and self.from_id == other.from_id and self.to_id == other.to_id

    def reverse(self):
        return Edge(self.id, self.to_id, self.from_id, self.properties)


class Graph:
    def __init__(self):
        self.vertices = {}
        self.edges = {}
        self.vertices_id = 1
        self.edges_id = 1

    def __str__(self):
        ret = "Graph{\nVertices:\n"
        for k in self.vertices:
            ret += "  " + str(self.vertices[k]) + ",\n"
        ret += "Edges:\n"
        for k in self.edges:
            ret += "  " + str(self.edges[k]) + ",\n"
        ret += "}\n"
        return ret

    def add_vertex(self, _id, properties):
        if _id in self.vertices:
            raise GraphError('DUPLICATE_VERTEX_ID')
        self.vertices[_id] = Vertex(_id, properties)
        return self.vertices[_id]

    def add_edge(self, edge):
        if edge.id in self.edges:
            raise GraphError('DUPLICATE_EDGE_ID')
        if edge.from_id not in self.vertices:
            # raise GraphError('NON_EXISTING_FROM_ID')
            log.error(f'Non-existing from id:{edge.from_id}')
            return
        if edge.to_id not in self.vertices:
            # raise GraphError('NON_EXISTING_TO_ID')
            log.error(f'Non-existing to id:{edge.to_id}')
            return
        self.edges[edge.id] = edge
        self.edges_id += 1
        self.vertices[edge.to_id].in_edges.append(edge.id)
        self.vertices[edge.from_id].out_edges.append(edge.id)
        return self.edges[edge.id]


class Comparer:
    def __init__(self, g1, g2):
        self.graph1 = g1
        self.graph2 = g2
