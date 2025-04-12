from ecat_repl.stuff import read_sdo, set_uri

set_uri('amax-5580.local:5555')
an = read_sdo(['Assigned_name'], [1])[1]['Assigned_name']
print(an)