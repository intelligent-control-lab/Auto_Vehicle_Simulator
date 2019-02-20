from glob import glob
import os
for f in glob('data/lane_changing_data/*.csv'):
    print(f)
    target_id = int(f.split('/')[-1].split('_')[0])
    print(target_id)
    os.system("python first_person_view.py " + f + ' ' + str(target_id))
