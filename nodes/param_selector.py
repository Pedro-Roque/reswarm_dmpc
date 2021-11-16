"""
    Param selector for generators - easy way to do "higher wP, lower wV, etc...
"""

# Select test type from:
# - 'ut' for Unitary test
# - 'fc' for Formation ctl
test_type = 'fc'

# Select profile type:
# Ut: [p, v, q, w, f, t]   , p,v,q,w = 0,1 , f,t = 0,1,2
# Fc: [rl, rf, v, q, w, f, t]   , v,q,w = 0,1 , rl,rf,f,t = 0,1,2
profile = [0, 1, 0, 1, 1, 2]
profile_fc = [1, 1, 1, 1, 0, 1, 2]

found = False
if test_type == 'ut':
    test_param = 50
    for wp in range(2):
        for wv in range(2):
            for wa in range(2):
                for ww in range(2):
                    for wf in range(3):
                        for wt in range(3):

                            if wp == wv or wa == ww:
                                continue

                            if wp == profile[0] and \
                               wv == profile[1] and \
                               wa == profile[2] and \
                               ww == profile[3] and \
                               wf == profile[4] and \
                               wt == profile[5]:
                                found = True
                                print("Param: ", test_param)
                                print("Param list: ", wp, wv, wa, ww, wf, wt)

                            test_param = test_param + 1

elif test_type == 'fc':
    test_param = 0
    for wr in range(3):
        for wp in range(3):
            for wv in range(2):
                for wa in range(2):
                    for ww in range(2):
                        for wf in range(3):
                            for wt in range(3):

                                if (wp == 0 and wv == 0) or wa == ww:
                                    continue

                                if wr == profile_fc[0] and \
                                   wp == profile_fc[1] and \
                                   wv == profile_fc[2] and \
                                   wa == profile_fc[3] and \
                                   ww == profile_fc[4] and \
                                   wf == profile_fc[5] and \
                                   wt == profile_fc[6]:
                                    found = True
                                    print("Param: ", test_param)
                                    print("Param list: ", wp, wv, wa, ww, wf, wt)

                                test_param = test_param + 1
else:
    print("Wrong test type!")
    exit()

if not found:
    print("Wrong profile selected. Make sure you don't have repeated weights.")
