import json
import requests


class APIMng:
    def __init__(self, melexID):
        self.Melex_ID = melexID
        self.Requests_endpoint = {
            'url_recived': 'http://213.97.17.253:9000/requests/state/recived',
            'url_put': 'http://213.97.17.253:9000/request',
            'url_progress': 'http://213.97.17.253:9000/requests/state/progress',
            'url_all_tasks' : 'http://213.97.17.253:9000/requests',
            'url_create_request' : 'http://213.97.17.253:9000/request',
            'json' : None,
            'json_create' : None,
            'id': None
        }
        self.ParametersCCAA_endpoint = {
            'url': 'http://213.97.17.253:9000/parametersCA/',
            'json': None
        }
        self.getCarstate_endpoint ={
            'url': "http://213.97.17.253:9000/parametersCA/name/Melex"
        }
        self.create_newStop = {
           "url_post" : "http://213.97.17.253:9000/stop",
           "json" : None
        }
        self.estate = None

    def __del__(self):
        pass



    # Pedir tareas REQUESTS/GET
    def get_requests_api(self):
        r = requests.get(url=self.Requests_endpoint['url_recived'])
        self.Requests_endpoint['json'] = json.loads(r.text)
        if isinstance(json.loads(r.text), dict):
            return []
        else:
            return json.loads(r.text)

    def get_cars_status(self, key):
        # print(self.getCarstate_endpoint['url'] + key[5])
        r = requests.get(url=self.getCarstate_endpoint['url'] + key[5])
        return json.loads(r.text)


    def task_progress(self):
        r = requests.get(url=self.Requests_endpoint['url_progress'])
        # print(r.text)
        if isinstance(json.loads(r.text), dict):
            return []
        else:
            return json.loads(r.text)

    def post_request_api(self):
        r = requests.post(self.Requests_endpoint['url_put'] , json=self.Requests_endpoint['json_create'])
        print(r.text)
    # Mandar actualización tareas (progreso o acabada) REQUESTS/PUT
    def put_requests_api(self):
        # print(self.Requests_endpoint['url_put'] + '/' + str(self.Requests_endpoint['id']), self.Requests_endpoint['json'])
        print("PETICION", self.Requests_endpoint['url_put'] + '/' + str(self.Requests_endpoint['id']))
        r = requests.put(self.Requests_endpoint['url_put'] + '/' + str(self.Requests_endpoint['id']), json=self.Requests_endpoint['json'])
        print(r.text)

    # Mandar parámetros del vehículo (PARAMETERScar/PUT)
    def put_ParametersCCAA_api(self):
        r = requests.put(self.ParametersCCAA_endpoint['url'] + self.Melex_ID, json=self.ParametersCCAA_endpoint['json'])

    # Crear JSON para hacer el put al endpoint Requests
    def requests_put_json(self, data, id):
        data['car'] = self.Melex_ID
        # print("data en request_put_json", data)
        self.Requests_endpoint["json"] = data
        self.Requests_endpoint["id"] = id

    def request_create_post_request(self, data):
        self.Requests_endpoint["json_create"] = data

    def create_ParametersCCAA_json(self, data):
        self.ParametersCCAA_endpoint["json"] = data

    def create_stop_json(self, data):
        self.create_newStop["json"] = data

    def stop_post(self):
        r = requests.post(self.create_newStop['url_post'], json=self.create_newStop['json'])
        print(r.text)

    def get_all_tasks(self):
        r = requests.get(url=self.Requests_endpoint['url_all_tasks'])
        # print(r.text)
        if isinstance(json.loads(r.text), dict):
            return None, False
        else:
            return json.loads(r.text), True

    def delete_tasks(self):
        r = requests.delete(self.Requests_endpoint['url_put'] + '/' + str(self.Requests_endpoint['id']))
        # print('TASK DELETED ', str(self.Requests_endpoint['id']) )



