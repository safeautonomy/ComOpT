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

from comopt.database.base import Operator
from comopt.property_graph import property
from loguru import logger as log


class Converter(Operator):

    def add_vertex(self, v):
        self.write(
            f'CREATE ({v.id}:{v.properties["type"]} {property.for_neo4j(v.id, property.for_printing(v.properties))})')

    def add_edge(self, e, f, t):
        self.write(
            f'MATCH (f:{f.properties["type"]}), (t:{t.properties["type"]})'
            f'WHERE f.name = "{f.id}" AND t.name = "{t.id}"'
            f'CREATE (f)-[{e.id}:{e.properties["type"]} {property.for_neo4j(e.id, property.for_printing(e.properties))}]->(t)'
        )

    def add_graph(self, g):
        cnt = 0
        for v in g.vertices:
            self.add_vertex(g.vertices[v])
            cnt += 1
            if cnt % 2000 == 0:
                log.info(f'add vertices: {cnt}/{len(g.vertices)}')
        log.info('finish adding vertices')

        cnt = 0
        for e in g.edges:
            ee = g.edges[e]
            self.add_edge(g.edges[e], g.vertices[ee.from_id], g.vertices[ee.to_id])
            cnt += 1
            if cnt % 2000 == 0:
                log.info(f'add edges: {cnt}/{len(g.edges)}')
        log.info('finish adding edges')

    def clear(self):
        self.write('MATCH (n) OPTIONAL MATCH (n)-[r]-() DELETE n,r')
