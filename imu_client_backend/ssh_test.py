from fabric import Connection
from tx_dict import tx_dict

# Define the router's IP address and login credentials
router_ip = '192.168.88.11'
router_username = 'admin'
beamangle = -7.3
if beamangle <= 90 or beamangle >= 90:
    if beamangle > 30:
        print('sector C')
        beamangle-= 60
    elif beamangle < -30:
        print('sector A')
        beamangle += 60
    else:
        print('sector B')
        
# Define the command to change tx sector
tx_val = tx_dict[beamangle]

print(f'tx_sector:{tx_val}')

router_command = f'/interface w60g> set 0 tx-sector={tx_val}'

# Connect to the router via SSH
with Connection(host=router_ip, user=router_username) as conn:
    # Execute the user-defined command on the router and print the output
    result = conn.run(router_command, hide=True)
    print(result.stdout) 