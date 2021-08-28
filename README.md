# Active_MRSLAM

## Introdução

**Autor**: Luiz Eugênio Santos Araújo Filho

**Título**: Multi-robot Autonomous Exploration and Map Merging in Unknown Environments

Este software está disponível nos seguintes sites:  
> **ITA FTP server**: ftp://labattmot.ele.ita.br/ele/lesaf/My_Software/  
> **GitHub**: https://github.com/Intelligent-Machines-Lab/Active_MRSLAM  

Obs.: Alternativas para acessar URLs com "ftp://":  
1. Em computadores com MS-Windows, abra o Windows Explorer (Windows key + E) e digite o URL na linha de endereços, como mostrado em:  
   - [How to use FTP via Windows Explorer](https://www.hostgator.com/help/article/how-to-use-ftp-via-windows-explorer),  
   - [How to Access an FTP Server in Windows Explorer](https://www.dummies.com/computers/operating-systems/windows-xp-vista/how-to-access-an-ftp-server-in-windows-explorer/).  
Copie o arquivo desejado para o disco local e o abra usando o aplicativo correspondente já instalado.  
2. Use um cliente ftp, p. ex., [FileZilla Client](https://filezilla-project.org/download.php?type=client).  
3. Use um web browser que aceite URLs com "ftp://", p. ex., [WaterFox](https://www.waterfox.net/).  

A Dissertação de Mestrado e outras publicações de Luiz Eugênio Santos Araújo Filho estão disponíveis em:  
> **ITA FTP server**: ftp://labattmot.ele.ita.br/ele/lesaf/My_Publications/  

## Pré-requisitos

Software utilizado nesse trabalho:
- MATLAB 2021a
- ROS Melodic
- Gazebo 9.0

O computador utilizado para execução e desenvolvimento foi um **Acer Nitro 5** com as seguintes especificações:
- **PROCESSADOR**: Intel Core i7-7700HQ de 7ª geração (2.8 GHz até 3.8 GHz) 6 MB de Cache
- **PLACA DE VÍDEO**: NVIDIA GeForce GTX 1050Ti com 4 GB gDDR5
- **MEMÓRIA RAM**: 16 GB DDR4 SDRAM 2.400 MHz
- **ARMAZENAMENTO**: HD 1 TB SATA III (6 GB/s) – 5.400 RPM

## Estrutura

As pastas contidas nesse repositório contém os códigos-fonte e estão prontos para serem baixados e utilizados uma vez que os programas pré requisitos estejam instalados.

### **isis_gazebo**

Contém o pacote ROS de mesmo nome. Deve ser colocado na pasta **catkin_ws/src**, proveniente da configuração de um ambiente **catkin** ao final da instalção do ROS. Para ser identificada pelo ROS, o comando **catkin_make** deve ser executado após a cópia do pacote.  

### **matlab_code**

Contém os arquivos **.m** e **.mat** necessários para execução do programa principal no MATLAB.

## Reprodução do trabalho

Para reproduzir os experimentos realizados no trabalho apenas dois scripts são executados no MATLAB: `test_autonomous_exploration.m` e `offline_rendezvous.m`, este último reproduz o experimento utilizando dados gravados da exploração no ambiente **LMI Corridor**.

O arquivo `test_autonomous_exploration.m` deve ser executado em conjunto com o ROS. Em caso de simulação, o pacote **isis_gazebo** deve ser iniciado através de um `roslaunch` em uma das opções listadas abaixo:

1. Small Environment
    - 1 robô: `isis_house_small_1.launch`
    - 3 robôs: `isis_house_small_3.launch` 
2. Large Environment
    - 1 robô: `isis_house_large_1.launch`
    - 3 robôs: `isis_house_large_3.launch` 
    - 5 robôs: `isis_house_large_5.launch`

Caso seja utilizado em um robô real, os seguintes tópicos devem estar presentes:
> - `/scan`: mensagem do tipo **LidarScan**
> - `/cmd_vel_isis`: mensagem do tipo **geometry_msgs/Twist**
> - `/rel_info_isis`: mensagem do tipo **std_msgs/Float64MultiArray**





