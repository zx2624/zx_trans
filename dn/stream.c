#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <signal.h>
#define PORT1 5000
#define PORT2 24000
void stop(int sig){
    puts("finish data transfer.");
    exit(0);
}
int main(){
    signal(SIGINT,stop);
    struct sockaddr_in serv1, serv2, client;
    bzero(&serv1,sizeof(serv1));
    serv1.sin_family=AF_INET;
    serv1.sin_addr.s_addr=inet_addr("10.21.100.152");// serv1 used to recv data and transfer.
    serv1.sin_port=htons(PORT1);
    int fd1 = socket(AF_INET,SOCK_DGRAM,0);
    if(fd1 == -1){
        perror("create socket fd1 error: ");
        exit(1);
    }
    if(bind(fd1,(struct sockaddr*)&serv1,sizeof(serv1))==-1){
        perror("bind serv1 ");
        exit(1);
    }
    bzero(&serv2,sizeof(serv2));
    serv2.sin_family=AF_INET;
    serv2.sin_addr.s_addr=inet_addr("10.21.100.153");// serv2 used to recv data for vlc display.
    serv2.sin_port=htons(PORT2);

    void *buff = malloc(1500); // vlc number of data bits is 1316
    while(1){
        int addr_len = sizeof(client);
        memset(buff,0,1500);
        int ret = recvfrom(fd1,buff,1500,0,(struct sockaddr *)&client,&addr_len);
        printf("%5d ",ret);
        if(ret == -1){
           perror("recvfrom error: ");
           exit(1);
        }
        else {
           //ret = sendto(fd1,buff,strlen(buff),0,(struct sockaddr *)&serv2,sizeof(serv2)); // not fd2
           ret = sendto(fd1,buff,ret,0,(struct sockaddr *)&serv2,sizeof(serv2)); // not fd2
           printf("%5d\n",ret);
           if(ret == -1){
               perror("sendto error: ");
               exit(1);
           }
        }
    }
    return 0;
}
