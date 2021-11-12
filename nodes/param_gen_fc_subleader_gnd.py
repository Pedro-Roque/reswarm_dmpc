
# Generator for formation tests - subleader
import json

traj_profiles = [1, 2, 3]

# Test 1 components
expiration_time = [120, 60, 60]
bearings = {'l': [0.0, -0.8, 0.0], 'f1': [-1.0, 0.0, 0.0]}
qd = [0.707, -0.707, 0, 0]

wr_l = [5, 20, 100]
wp_l = [0.5, 5, 100]
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

                                yaml_write = {'expiration_time': exp_t}
                                yaml_write['W'] = W_l
                                yaml_write['WN'] = W_N_l
                                yaml_write['WN_gain'] = 200
                                yaml_write['WN_gain'] = 200
                                yaml_write['bearings'] = bearings
                                yaml_write['qd'] = qd

                                file_name = 'subleader_iface_gnd_3' + str(profile) + "{:03d}".format(setting_counter)

                                with open('/home/roque/reswarm_ws/src/reswarm_dmpc/config/bumble/' + file_name + '.yaml', 'w') as file:
                                    json.dump(yaml_write, file)

                                setting_counter = setting_counter + 1
                                print(file_name)
