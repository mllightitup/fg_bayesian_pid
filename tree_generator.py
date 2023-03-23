from pprint import pprint
from flightgear_python.fg_if import PropsConnection

props_conn = PropsConnection('localhost', 5500)
props_conn.connect()  # Make an actual connection
pprint(props_conn.list_props('/autopilot', recurse_limit=6))  # List the top-level properties, no recursion
