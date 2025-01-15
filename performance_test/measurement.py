"""
ノード名を2つ（PubとSub）指定すれば、そのノードの間の通信性能を表示してくれる。
1. 指定された2つのノードが本当につながっているかを判定
2. ノード間のレイテンシを測定
3. ノード間のスループットを測定
4. 通信性能一覧をtxtファイルに書き出す

（本当はやりたいこと）
2つのノード間に複数のルートがあるなら、それを洗い出す → 現状は末端のpubとsub間しか測定できない
全てのルートに対して各評価指標を算出
"""

import sys
import os
import shutil

# metadataから、2つのノードがつながっているかを判定
# cmd_args -> dict, dict, list[str]
def check_connect(args):
    # 結果格納用
    pub_topic_list = []
    sub_topic_list = []

    pub_node_name = args[1]
    sub_node_name = args[2]
    pub_metadata_path = f"logs/{pub_node_name}_log/metadata.txt"
    sub_metadata_path = f"logs/{sub_node_name}_log/metadata.txt"

    with open(pub_metadata_path, "r") as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith("NodeType:"):
                pub_node_type = line.split(":", 1)[1].strip().split(",")[0] # ”Publisher", "Intermediate" など
            if line.startswith("PayloadSize:"):
                pub_payload_size = line.split(":", 1)[1].strip().split(",")[0] 
            if line.startswith("Period:"):
                pub_period_ms = line.split(":", 1)[1].strip().split(",")[0] 

        if(pub_node_type == "Publisher"):
            for line in lines:
                if line.startswith("Topics:"):
                    topics = line.split(":", 1)[1].strip().split(",")

        elif(pub_node_type == "Intermediate"):
            for line in lines:
                if line.startswith("Topics(Pub):"):
                    topics = line.split(":", 1)[1].strip().split(",")

        pub_topic_list = [topic for topic in topics if topic]

    with open(sub_metadata_path, "r") as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith("NodeType:"):
                sub_node_type = line.split(":", 1)[1].strip().split(",")[0]

        if(sub_node_type == "Subscriber"):
            for line in lines:
                if line.startswith("Topics:"):
                    topics = line.split(":", 1)[1].strip().split(",")

        elif(sub_node_type == "Intermediate"):
            for line in lines:
                if line.startswith("Topics(Sub):"):
                    topics = line.split(":", 1)[1].strip().split(",")

        sub_topic_list = [topic for topic in topics if topic]

    # 共通のトピックのリスト
    common_topics = list(set(pub_topic_list) & set(sub_topic_list))
    if len(common_topics) > 0:
        print(f"Connection Success! 共通のトピック: {common_topics}")
    else:
        raise ValueError("Connection failed")
    
    pub_info = {}
    sub_info = {}
    pub_info["name"] = pub_node_name
    pub_info["type"] = pub_node_type
    pub_info["payload_size"] = pub_payload_size
    pub_info["period_ms"] = pub_period_ms
    sub_info["name"] = sub_node_name
    sub_info["type"] = sub_node_type

    print(pub_info)
    print(sub_info)

    return pub_info, sub_info, common_topics


# 2つのノードの共通トピックそれぞれに対し、ログファイルを取得し、辞書形式で保存
# dict, dict, list[str] -> dict, dict
def get_logdata(pub_info, sub_info, topic_list):
    pub_node_name = pub_info["name"]
    pub_logdata = {}

    if(pub_info["type"] == "Publisher"):
        for topic in topic_list:
            pub_logdata[f"{topic}"] = []
            pub_logdata_path = f"logs/{pub_node_name}_log/{topic}_log.txt"

            with open(pub_logdata_path, "r") as log_file:
                lines = log_file.readlines()
                for line in lines:
                    line = line.strip()
                    if "StartTime:" in line:
                        start_time = line.split(":", 1)[1].strip().split(",")[0]
                        pub_logdata[f"{topic}"].append(("StartTime", start_time))
                    if "EndTime:" in line:
                        end_time = line.split(":", 1)[1].strip().split(",")[0]
                        pub_logdata[f"{topic}"].append(("EndTime", end_time))

                    if "Index:" in line and "Timestamp:" in line:
                        # "Index:" と "Timestamp:" を分割して値を取得
                        parts = line.split(", ")
                        index = int(parts[0].split(":")[1].strip())
                        timestamp = int(parts[1].split(":")[1].strip())
                        pub_logdata[f"{topic}"].append((index, timestamp))
    elif(pub_info["type"] == "Intermediate"):
        for topic in topic_list:
            pub_logdata[f"{topic}"] = []
            pub_logdata_path = f"logs/{pub_node_name}_log/{topic}_pub_log.txt"

            with open(pub_logdata_path, "r") as log_file:
                lines = log_file.readlines()
                for line in lines:
                    line = line.strip()
                    if "StartTime:" in line:
                        start_time = line.split(":", 1)[1].strip().split(",")[0]
                        pub_logdata[f"{topic}"].append(("StartTime", start_time))
                    if "EndTime:" in line:
                        end_time = line.split(":", 1)[1].strip().split(",")[0]
                        pub_logdata[f"{topic}"].append(("EndTime", end_time))

                    if "Index:" in line and "Timestamp:" in line:
                        # "Index:" と "Timestamp:" を分割して値を取得
                        parts = line.split(", ")
                        index = int(parts[1].split(":")[1].strip())
                        timestamp = int(parts[2].split(":")[1].strip())
                        pub_logdata[f"{topic}"].append((index, timestamp))

    sub_node_name = sub_info["name"]
    sub_logdata = {}

    if(sub_info["type"] == "Subscriber"):
        for topic in topic_list:
            sub_logdata[f"{topic}"] = []
            sub_logdata_path = f"logs/{sub_node_name}_log/{topic}_log.txt"

            with open(sub_logdata_path, "r") as log_file:
                lines = log_file.readlines()
                for line in lines:
                    line = line.strip()
                    if "StartTime:" in line:
                        start_time = line.split(":", 1)[1].strip().split(",")[0]
                        sub_logdata[f"{topic}"].append(("StartTime", start_time))
                    if "EndTime:" in line:
                        end_time = line.split(":", 1)[1].strip().split(",")[0]
                        sub_logdata[f"{topic}"].append(("EndTime", end_time))

                    if "Index:" in line and "Timestamp:" in line:
                        # "Index:" と "Timestamp:" を分割して値を取得
                        parts = line.split(", ")
                        index = int(parts[0].split(":")[1].strip())
                        timestamp = int(parts[1].split(":")[1].strip())
                        sub_logdata[f"{topic}"].append((index, timestamp))
    elif(sub_info["type"] == "Intermediate"):
        for topic in topic_list:
            sub_logdata[f"{topic}"] = []
            sub_logdata_path = f"logs/{sub_node_name}_log/{topic}_sub_log.txt"

            with open(sub_logdata_path, "r") as log_file:
                lines = log_file.readlines()
                for line in lines:
                    line = line.strip()
                    if "StartTime:" in line:
                        start_time = line.split(":", 1)[1].strip().split(",")[0]
                        sub_logdata[f"{topic}"].append(("StartTime", start_time))
                    if "EndTime:" in line:
                        end_time = line.split(":", 1)[1].strip().split(",")[0]
                        sub_logdata[f"{topic}"].append(("EndTime", end_time))

                    if "Index:" in line and "Timestamp:" in line:
                        # "Index:" と "Timestamp:" を分割して値を取得
                        parts = line.split(", ")
                        index = int(parts[1].split(":")[1].strip())
                        timestamp = int(parts[2].split(":")[1].strip())
                        sub_logdata[f"{topic}"].append((index, timestamp))

    # print("pub_logdata\n", pub_logdata)
    # print("sub_logdata\n", sub_logdata)
            
    return pub_logdata, sub_logdata

# Pubのタイムスタンプ一覧とSubのタイムスタンプ一覧を受け取り、その差の様々な統計データを算出し、resultsフォルダにtxtとして出力
def measure_latency(pub_logdata, sub_logdata, topic_list):
    latency_results = {}

    # resultsディレクトリに延々と追記されないように毎回リセット
    try:
        shutil.rmtree("results")
        print(f"delete directory: results/")
    except FileNotFoundError:
        print(f"指定されたディレクトリが存在しません: results")
    except PermissionError:
        print(f"ディレクトリの削除に失敗しました（権限不足）: results")
    
    os.makedirs("results", exist_ok=True)

    for topic in topic_list:
        latency_results[f"{topic}"] = []

        pub_start_time = next(item[1] for item in pub_logdata[f"{topic}"] if item[0] == "StartTime")
        pub_end_time = next(item[1] for item in pub_logdata[f"{topic}"] if item[0] == "EndTime")
        sub_start_time = next(item[1] for item in sub_logdata[f"{topic}"] if item[0] == "StartTime")
        sub_end_time = next(item[1] for item in sub_logdata[f"{topic}"] if item[0] == "EndTime")
        # StartTimeとEndTimeは用済なので消す
        pub_logdata[f"{topic}"] = [item for item in pub_logdata[f"{topic}"] if item[0] != "StartTime" and item[0]!= "EndTime"]
        sub_logdata[f"{topic}"] = [item for item in sub_logdata[f"{topic}"] if item[0] != "StartTime" and item[0]!= "EndTime"]

        # この共通集合に入る時間帯が計測対象
        common_start_time = int(max(pub_start_time, sub_start_time))
        common_end_time = int(min(pub_end_time, sub_end_time))

        # Start~Endの共通集合に入らないindexを除く
        pub_indices = {item[0] for item in pub_logdata[f"{topic}"] if int(item[1]) >= common_start_time and int(item[1]) <= common_end_time}
        sub_indices = {item[0] for item in sub_logdata[f"{topic}"] if int(item[1]) >= common_start_time and int(item[1]) <= common_end_time}
        print(pub_indices)
        print(sub_indices)

        # その上で片方にしか入っていないindexをlossとしてloss率を計算
        los_index_count = len(set(pub_indices) - set(sub_indices)) + len(set(pub_indices) - set(sub_indices))
        print(los_index_count)
        common_indices = pub_indices.intersection(sub_indices)
        los_index_rate = los_index_count / (len(common_indices) + los_index_count)

        for index in common_indices:
            timestamp_pub = next(timestamp for idx, timestamp in pub_logdata[f"{topic}"] if idx == index)
            timestamp_sub = next(timestamp for idx, timestamp in sub_logdata[f"{topic}"] if idx == index)

            latency_results[f"{topic}"].append((index, (timestamp_sub - timestamp_pub)/ 1_000_000))

        with open("results/latency_results.txt", "a") as f:
            f.write(f"topic: {topic}\n")
            f.write(f"loss: {los_index_rate}[%]\n")
            for index, latency in latency_results[f"{topic}"]:
                f.write(f"Index: {index}, Latency: {latency}ms\n")

    print("complete caluculating latency!")
    return latency_results

def measure_throughput(pub_logdata, sub_logdata, topic_list):
    return

if __name__ == "__main__":
    args = sys.argv
    pub_info, sub_info, common_topics = check_connect(args)
    pub_logdata, sub_logdata = get_logdata(pub_info, sub_info, common_topics)
    latency_results = measure_latency(pub_logdata, sub_logdata, common_topics)