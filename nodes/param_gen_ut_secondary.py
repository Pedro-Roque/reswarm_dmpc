# Generator for unit tests - secondary
import json

test_list = [1, 2]

# Test 1 components
unit_test_time = [45, 45]
expiration_time = [310, 310]
targets_t1 = {'t':
                    {
                    't1': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                    't2': [10.95, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0], 
                    't3': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                    't4': [10.75, -7.75, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                    't5': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0], 
                    't6': [10.75, -7.5, 4.55, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                    't7': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
                    }
                }
             
targets_t2 = {'q':
                    {
                    't1': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                    't2': [10.75, -7.5, 4.3, 0, 0, 0, 0.259, 0, 0, 0.966, 0, 0, 0],
                    't3': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                    't4': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0.259, 0, 0.966, 0, 0, 0] ,
                    't5': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0] ,
                    't6': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0.259, 0.966, 0, 0, 0],
                    't7': [10.75, -7.5, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
                    }
                }
              
wp_l = [10, 100]
wv_l = [10, 100]
wa_l = [10, 100]
ww_l = [10, 100]

wf_l = [5, 25, 50]
wt_l = [5, 25, 50]

for test in test_list:
    print("Test: ", test)
    setting_counter = 50
    for wp in wp_l:
        for wv in wv_l:
            for wa in wa_l:
                for ww in ww_l:
                    for wf in wf_l:
                        for wt in wt_l:

                            if wp == wv or wa == ww:
                                continue


                            utt = unit_test_time[test-1]
                            exp_t = expiration_time[test-1]
                            if test == 1:
                                test_type = 'translation'
                                target = targets_t1
                            else:
                                test_type = 'attitude'
                                target = targets_t2
                            
                            wp_v = [wp]*3
                            wv_v = [wv]*3
                            wa_v = [wa]*3
                            ww_v = [ww]*3

                            wf_v = [wf]*3
                            wt_v = [wt]*3

                            W_l = wp_v + wv_v + wa_v + ww_v + wf_v + wt_v
                            W_N_l = wp_v + wv_v + wa_v + ww_v

                            yaml_write = {'test_num': test}
                            yaml_write['unit_test_time'] = utt
                            yaml_write['expiration_time'] = exp_t
                            yaml_write['W'] = W_l
                            yaml_write['WN'] = W_N_l
                            yaml_write['WN_gain'] = 200
                            yaml_write['WN_gain'] = 200
                            yaml_write['targets'] = target

                            file_name = 'test_'+test_type+'_iss_'+str(test)+"{:02d}".format(setting_counter)

                            with open('/home/roque/reswarm_ws/src/reswarm_dmpc/config/bumble/'+file_name+'.yaml', 'w')  as file:
                                json.dump(yaml_write, file)

                            setting_counter = setting_counter + 1
                            print("Settings counter: ", setting_counter)
                            if setting_counter > 99:
                                print("Oooops")
                                break