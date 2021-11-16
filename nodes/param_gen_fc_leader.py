
# Generator for formation tests - subleader
import json

traj_profiles = [1, 2, 3]

# Test 1 components
unit_test_time = [60, 60, 60]
expiration_time = [120, 60, 60]
bearings = {'f1': [-1.0, 0.0, 0.0]}
traj_start = [10.75, -8.5, 4.3]
qd = [0, 0, -0.707, 0.707]
traj_velocity_1 = {'t1': [0, -0.01, 0], 't2': [0.03, -0.005, 0.03]}
traj_velocity_2 = {'t2': [0.03, -0.005, 0.03]}
traj_velocity_3 = {'t1': [0, -0.01, 0]}

wr_l = [5, 20, 100]
wp_l = [10, 25, 50]

wv_l = [10, 100]
wa_l = [10, 100]
ww_l = [10, 100]

wf_l = [5, 25, 50]
wt_l = [5, 25, 50]

for profile in traj_profiles:
    print("Test: ", profile)
    setting_counter = 0
    for wr in wr_l:
        for wp in wp_l:
            for wv in wv_l:
                for wa in wa_l:
                    for ww in ww_l:
                        for wf in wf_l:
                            for wt in wt_l:

                                if wp == wv or wa == ww:
                                    continue

                                utt = unit_test_time[profile - 1]
                                exp_t = expiration_time[profile - 1]

                                wr_v = [wr] * 3
                                wp_v = [wp] * 3
                                wv_v = [wv] * 3
                                wa_v = [wa] * 3
                                ww_v = [ww] * 3

                                wf_v = [wf] * 3
                                wt_v = [wt] * 3

                                W_l = wr_v + wp_v + wv_v + wa_v + ww_v + wf_v + wt_v
                                W_N_l = wr_v + wp_v + wv_v + wa_v + ww_v

                                yaml_write = {'test_time': utt}
                                yaml_write['expiration_time'] = exp_t
                                yaml_write['W'] = W_l
                                yaml_write['WN'] = W_N_l
                                yaml_write['WN_gain'] = 200
                                yaml_write['WN_gain'] = 200
                                yaml_write['bearings'] = bearings
                                yaml_write['qd'] = qd
                                yaml_write['traj_start'] = traj_start

                                qd = [0, 0, -0.707, 0.707]
                                if profile == 1:
                                    yaml_write['traj_velocity'] = traj_velocity_1
                                elif profile == 2:
                                    yaml_write['traj_velocity'] = traj_velocity_2
                                elif profile == 3:
                                    yaml_write['traj_velocity'] = traj_velocity_3

                                file_name = 'leader_iface_iss_3' + str(profile) + "{:03d}".format(setting_counter)

                                with open('/home/roque/reswarm_ws/src/reswarm_dmpc/config/honey/' + file_name + '.yaml', 'w') as file:
                                    json.dump(yaml_write, file)

                                setting_counter = setting_counter + 1
                                print(file_name)
