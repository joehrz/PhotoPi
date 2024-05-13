import json

class Config:
    def __init__(self, filepath='params.json'):
        self.filepath = filepath
        self.data = self.load_config()

    def load_config(self):
        try:
            with open(self.filepath, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return {}  # Return an empty dict if the file doesn't exist

    def save_config(self):
        with open(self.filepath, 'w') as f:
            json.dump(self.data, f, indent=4)

    def get_value(self, key):
        return self.data.get(key, None)

    def set_value(self, key, value):
        self.data[key] = value