from pprint import pprint
from flightgear_python.fg_if import PropsConnection

props_conn = PropsConnection('localhost', 5500)
props_conn.connect()  # Make an actual connection
pprint(props_conn.list_props('/orientation', recurse_limit=9))  # List the top-level properties, no recursion

 # 'properties': {'/fdm/jsbsim/moments/l-aero-lbsft': 320.3931626,
 #                '/fdm/jsbsim/moments/l-external-lbsft': 0.0,
 #                '/fdm/jsbsim/moments/l-gear-lbsft': 0.0,
 #                '/fdm/jsbsim/moments/l-prop-lbsft': -320.3931456,
 #                '/fdm/jsbsim/moments/l-total-lbsft': 1.695231418e-05,
 #                '/fdm/jsbsim/moments/m-aero-lbsft': -210.8798746,
 #                '/fdm/jsbsim/moments/m-external-lbsft': 0.0,
 #                '/fdm/jsbsim/moments/m-gear-lbsft': 0.0,
 #                '/fdm/jsbsim/moments/m-prop-lbsft': 210.8829583,
 #                '/fdm/jsbsim/moments/m-total-lbsft': 0.003083623065,
 #                '/fdm/jsbsim/moments/n-aero-lbsft': 32.84112362,
 #                '/fdm/jsbsim/moments/n-external-lbsft': 0.0,
 #                '/fdm/jsbsim/moments/n-gear-lbsft': 0.0,
 #                '/fdm/jsbsim/moments/n-prop-lbsft': -32.8411536,
 #                '/fdm/jsbsim/moments/n-total-lbsft': -2.997499258e-05,
 #                '/fdm/jsbsim/moments/pitch-stab-aero-lbsft': -210.8798746,
 #                '/fdm/jsbsim/moments/pitch-wind-aero-lbsft': -211.82195,
 #                '/fdm/jsbsim/moments/roll-stab-aero-lbsft': 319.3027128,
 #                '/fdm/jsbsim/moments/roll-wind-aero-lbsft': 318.6785299,
 #                '/fdm/jsbsim/moments/yaw-stab-aero-lbsft': 42.14374994,
 #                '/fdm/jsbsim/moments/yaw-wind-aero-lbsft': 42.14374994}}


