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

class List:
    def __init__(self, search_label, search_result):
        self.names = []
        for r in search_result:
            if r[search_label] is None:
                continue
            self.names.append(r[search_label]['name'])

    def limitation(self):
        return {
            'name': self.names
        }

    def unique(self):
        self.names = list(set(self.names))
        return self.names

    def empty(self):
        return len(self.names) == 0

def list_to_map(key, value):
    ret = dict()
    for i in range(len(key.names)):
        if key.names[i] not in ret:
            ret[key.names[i]] = set()
        ret[key.names[i]].add(value.names[i])
    return ret

    # def in_condition(self, label):
    #     ret = f'WHERE {label}.name IN ['
    #     for n in self.names:
    #         ret+=f'"{n}", '
    #     ret += ']'
    #     return ret
