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

from comopt.property_graph import graph

edge_book = {}


def add_edge_appearing_twice(g, edge):
    # get graph_created
    try:
        g.add_edge(edge)
    except graph.GraphError as existing:  # graph: created
        if existing.code != 'DUPLICATE_EDGE_ID':
            raise existing
        else:
            graph_created = True
    else:  # graph: not created
        graph_created = False

    # get book_count
    if edge.id not in edge_book.keys():
        book_count = 0
    else:
        book_count = edge_book[edge.id]

    # check graph and book consistent
    if (graph_created and book_count == 0) or (not graph_created and book_count > 0):
        raise NameError(
            f'Edge<{edge.id}> graph and book inconsistent, graph_created={graph_created}, book_count={book_count}')

    # book_count = 0: new edge, set book = 1
    if book_count == 0:
        edge_book[edge.id] = 1
    # book_count = 1: check if provided and existing edges are the same
    elif book_count == 1:
        existing = g.edges[edge.id]
        provided = edge
        if not existing.cmp(provided):
            raise NameError(f'Edge<{edge.id}> provided and existing inconsistent, '
                            f'provided:{provided}, '
                            f'existing:{existing}')
        edge_book[edge.id] = 2
    # book_count > 1: shouldn't happen, raise error
    elif book_count > 1:
        raise NameError(f'Edge<{edge.id}> appear more than twice')
