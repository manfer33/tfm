import datetime, gym, re

def save_json(q_values):
    f = open('/home/manfer/tfm_ws/src/tfm/arm_training/training_results/'+str(datetime.datetime.now())+'.txt', 'w')
    f.write(str(q_values))
    f.close()

def load_json(path):
    q_values = {}
    f = open(path, 'r')
    raw = f.read().lstrip("{").rstrip("}")
    parsed = re.findall('\[[^\]]*\]|\([^\)]*\)|\"[^\"]*\"|\S+',raw)
    temp, sa, state, action = 0, 0, 0, 0

    for i in range(len(parsed)):
        value = parsed[i].lstrip("('").rstrip(")").rstrip(",")
        if temp == 0:
            sa = value.split(',')
            state = sa[0].rstrip("'")
            action = int(sa[1])
            temp += 1
        elif temp == 1:
            temp += 1
        elif temp == 2:
            # print(value)
            q_values[(state, action)] = float(value)
            temp = 0
    return q_values
    