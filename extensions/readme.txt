注意：
这个移植的cartographer是release版本，修改了cartographer的工程结构：
包括：
1.去除了GMOCK，删除了所有test文件，因此，不编译gtest文件，加快了编译时间
2.去除了cloud，cloud是用于通讯的gRPC和Prometheus监控，我们用不上
3.修改了绝大部分的cmake文件，使之更加简洁明了，还去除了安装选项，因此不支持安装
