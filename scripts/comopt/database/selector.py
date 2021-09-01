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
from comopt.property_graph.property import for_where_condition
from loguru import logger as log


class Selector(Operator):

    def with_type(self, object_type):
        return self.read(f'MATCH (obj:{object_type}) RETURN obj')

    def with_properties(self, object_type, properties):
        return self.read(f'MATCH (obj:{object_type}) WHERE {for_where_condition("obj", properties)} RETURN obj')

    def with_format(self, fmt, limitations, results, debug=False):
        where = ''
        if type(limitations) is dict:
            for k in limitations.keys():
                where += f' AND {for_where_condition(k, limitations[k])}'
            where = f'WHERE {where[4:]}'
        elif type(limitations) is str:
            where = f'WHERE {limitations}'
        elif limitations is None:
            where = ''
        else:
            raise NameError('Unknown type of limitations')
        rets = ''
        for r in results:
            rets += f'{r}, '
        cmd = f'MATCH {fmt} {where} RETURN {rets[:-2]}'
        if debug:
            log.info(f'db read(with_format): {cmd}')
        return self.read(cmd)

    def with_formats(self, fmts, optionals, limitations, results):
        wheres = []
        for ll in limitations:
            where = ''
            for k in ll.keys():
                where += f' AND {for_where_condition(k, ll[k])}'
            wheres.append(where)
        rets = ''
        for r in results:
            rets += f'{r}, '
        cmd = ''
        for i in range(0, len(fmts)):
            if optionals[i]:
                cmd += 'OPTIONAL MATCH '
            else:
                cmd += 'MATCH '
            cmd += f'{fmts[i]} WHERE {wheres[i][4:]} '
        cmd += f'RETURN {rets[:-2]}'
        return self.read(cmd)
