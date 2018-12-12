from jsonschema import validate
import sys
import json

if(len(sys.argv) == 1):
	print("Please provide json file path to validate")
	exit()

jsonfile = sys.argv[1]

f = open("json_schema.json", "r")
schema = json.loads(f.read())
f.close()

f = open(jsonfile, "r")
data = json.loads(f.read())
f.close()

print validate(data, schema)
