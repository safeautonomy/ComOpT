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

import argparse
from loguru import logger as log
from comopt.map_parse.map_parse import map_parse, parse_junction_classify, parse_junction_cluster

parser = argparse.ArgumentParser()
parser.add_argument('name', help='map file name')
parser.add_argument('-u', '--update', help='ignore buffer and rerun', action="store_true")
group = parser.add_mutually_exclusive_group()
group.add_argument("--classify", help='only classify map elements', action="store_true")
group.add_argument("--cluster", help='only isomorphic clustering of map elements', action="store_true")

args = parser.parse_args()
if args.update:
    log.debug(f'Start parsing map "{args.name}", ignoring buffer.')
else:
    log.debug(f'Start parsing map "{args.name}", using buffer.')
if args.classify:
    parse_junction_classify(args.name, args.update, args.update)
elif args.cluster:
    parse_junction_cluster(args.name, args.update, args.update)
else:
    map_parse(args.name, args.update)
log.debug(f'Map parsing finished, result buffer is stored in the output folder.')
