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

from neo4j import GraphDatabase
from os import environ

def run_cypher(tx, cmd, ret):
    # print(f'Running: {cmd}')
    for r in tx.run(cmd):
        ret.append(r)


class Operator:
    def __init__(self, endpoint="neo4j://localhost:11005", auth=("neo4j", "neo4jneo4j")):


        self.driver = None
        if ((environ.get("NEO4J_ENDPOINT") is not None ) and (environ.get("NEO4J_USERNAME") is not None ) and (environ.get("NEO4J_PW") is not None )):
             self.driver = GraphDatabase.driver(environ.get("NEO4J_ENDPOINT"), auth=(environ.get("NEO4J_USERNAME"), environ.get("NEO4J_PW") ))
        else:
            self.driver = GraphDatabase.driver(endpoint, auth=auth)
        self.session = None

    def close(self):
        self.driver.close()

    def open_session(self):
        if self.session is None:
            self.session = self.driver.session()

    def close_session(self):
        if self.session is not None:
            self.session.close()
            self.session = None

    def read(self, cmd):
        ret = []
        self.session.read_transaction(run_cypher, cmd, ret)
        return ret

    def write(self, cmd):
        ret = []
        self.session.write_transaction(run_cypher, cmd, ret)
        return ret
