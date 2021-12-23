//server listen addr
char server_addr_list[5][30]={
    "tcp://*:5555",
    "tcp://*:6666",
    "tcp://*:5050",
    "tcp://*:6060",
    "tcp://*:6565"
};
//client connect addr
//replace the localhost to the actual server addr
//keep the port coinstency with that in server_addr_list
char connect_addr_list[5][30]={
    "tcp://localhost:5555",
    "tcp://120.24.248.93:6666",
    "tcp://120.24.248.93:5050",
    "tcp://120.24.248.93:6060",
    "tcp://120.24.248.93:6565"
};
//client subscribe addr
//replace the localhost to the actual server addr
char subaddr[30]="tcp://localhost:7050";
//server publish addr
char pubaddr[30]="tcp://*:7050";

// fixed IP address of clients
char pub_addr[30] = "tcp://*:18000";

char sub_addr_list[21][30] = {
    "tcp://localhost:18000", 
    "tcp://10.8.0.11:18000", 
    "tcp://10.8.0.13:18000",
    "tcp://10.8.0.15:18000",
    "tcp://10.8.0.17:18000",
    "tcp://10.8.0.19:18000",
    "tcp://10.8.0.21:18000",
    "tcp://10.8.0.23:18000",
    "tcp://10.8.0.25:18000",
    "tcp://10.8.0.27:18000",
    "tcp://10.8.0.29:18000",
    "tcp://10.8.0.31:18000",
    "tcp://10.8.0.33:18000",
    "tcp://10.8.0.35:18000",
    "tcp://10.8.0.37:18000",
    "tcp://10.8.0.39:18000",
    "tcp://10.8.0.41:18000",
    "tcp://10.8.0.43:18000",
    "tcp://10.8.0.45:18000",
    "tcp://10.8.0.47:18000",
    "tcp://10.8.0.49:18000"
};

char pub_ground_addr[30] = "tcp://*:18001";

char sub_ground_addr_list[20][30] = {
    "tcp://localhost:18001", 
    "tcp://10.8.0.13:18001",
    "tcp://10.8.0.15:18001",
    "tcp://10.8.0.17:18001",
    "tcp://10.8.0.19:18001",
    "tcp://10.8.0.21:18001",
    "tcp://10.8.0.23:18001",
    "tcp://10.8.0.25:18001",
    "tcp://10.8.0.27:18001",
    "tcp://10.8.0.29:18001",
    "tcp://10.8.0.31:18001",
    "tcp://10.8.0.33:18001",
    "tcp://10.8.0.35:18001",
    "tcp://10.8.0.37:18001",
    "tcp://10.8.0.39:18001",
    "tcp://10.8.0.41:18001",
    "tcp://10.8.0.43:18001",
    "tcp://10.8.0.45:18001",
    "tcp://10.8.0.47:18001",
    "tcp://10.8.0.49:18001"
};

// char pub_addr[4][30] = {
//     "tcp://*:18000", 
//     "tcp://*:18001", 
//     "tcp://*:18002",
//     "tcp://*:18003"
// };

// char sub_addr_list[4][30] = {
//     "tcp://10.8.0.11:18000", 
//     "tcp://10.8.0.11:18001", 
//     "tcp://10.8.0.11:18002",
//     "tcp://10.8.0.11:18003"
// };
