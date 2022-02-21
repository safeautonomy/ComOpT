from flask import Flask, request, jsonify, redirect
from flask_cors import *
import json
from google.protobuf.json_format import MessageToJson
import os

from comopt.geometry import Vector

CUR_DIR = os.path.dirname(__file__)

class MapVisualizerServer:
    def __init__(self, map):
        self.map = map

        app = Flask('MapVisualizer', static_url_path='/', static_folder=os.path.join(CUR_DIR, 'dist'))    
        
        CORS(app, supports_credentials=True)

        @app.route('/')
        def index():
            return redirect('/index.html')

        @app.route('/map')
        def get_map():
            return json.loads(MessageToJson(self.map.map))

        @app.route('/get_closest_lane/')
        def get_closest_lane():
            x = float(request.args.get('x'))
            y = float(request.args.get('y'))
            return jsonify({
                'id': map.get_closest_lane(Vector(x, y)).obj.id
            })

        app.logger.disabled = True
        
        self.app = app

    def run(self):
        self.app.run()