TIM5->Prescaler=8400-1
TIM5->PerIOD=1000-1
LED;

TIM8->Prescaler=168-1
TIM8->Period=20000-1
50HZ ���

TIM10->Prescaler=0
TIM->Period=5000-1
imu temperature control

drawer 380000

//���̿���ʹ�ü��ٶȼƣ���ʶ�𣬱����������˲�
//�ȳ���һ���ñ�������Ч�����������룬ʹ�������ȱ�ٵ��������������У�ͨ������������һ�������ݷ�Χ�Ƿ����
//800hz���ٶȼ�
//400hz������
//200hz������

//���ж����������̬
struct CAR{
	float accel[3];
	float gyro[3];//�������ʵ�ʵ���̬
	float mag[3];
	float integral_accel[3];
	float integral_gyro[3];
	float displacement[3];//������ʼλ�õ�λ��
	float mag_begin[3];//��ʼλ�õ�mag�Ƕ�
}car;
//��¼��ʼ����̬


//�������ߵ�Э�飺
Order_number	Move_type
num				1(��x������)	delta_x(��x�����)
				2(��y������)	delta_y(��y�����)
				3(б����)		delta_x,delta_y
				4(ת��)			rad
				5(����ƽ��)		
			
x,y���ƽ�棬�ü��ٶȼƺ͵����������������




//��˼����������
https://blog.csdn.net/qingfengxd1/article/details/106027819

//��Ԫ��������

https://www.zhihu.com/question/38298130


//���⣬������ٶȽǶ�����

//�ָ���ʵ�����ѡ�������ַ�����
һ��������+�Ҷȴ�����+��������//code_enable
�����жϵ㣺
a.��һ���ߣ���������¼�������㣩,ǰ�У�1��3
b.�ڶ����ߣ��������������㣬�����ƶ���1��3
c.�������ߣ��������������㣬ʹ�ó������ж��Ƿ�Ϊȡ������2
d.z����λ��������ȡ��
e.����



�����ߵ�������������




//ȥ����Ԫ����̬����

//�ؿ�Эͬ���ٶ����λ��pidʹ��

//������˼·��ͨ�������ٶ�������λ�ơ�������ָ����λ�ã�ͨ�������ĸ�����λ����������Ҫ���ڴ��µ���
//��������¼ת�ľ��룬Ȼ�����ٶ���ȷ������ı���
//���ߵ�Э��2.0��
ʹ��һ��n*6����������������
���������ǰ�������������ڼ���
i�����ڼ���
struct order step{
order[i][0]-->�˶�������:0ԭ��ֹͣ��1��x���˶���2��y���˶���3���������ƣ��ƶ�����4����ת��5�Ƿ����
	order[i][1]-->
	//1	vx,null,null,null,
	order[i][2]-->
	order[i][3]-->
	order[i][4]-->
	order[i][5]-->

}


//uint8_t	order_num
//int16_t	vx
//int16_t	vy
//float		w
//float		angle
//int 		displacement


//ͨ��Э��
to drawer:
//0b1111 1-e ������ͬ�ĳ�����0000-��һ���߶�	0101--�ڶ����߶�	1010--�������߶�	1111--���ĸ��߶�		
//0b1001 1-e ����
//0b0110 1-e ����

to car:
//ob1111 1-e ����ֹͣ��ͣ��
//0b1001 1-e ����
//0b0110 1-e ����

//��һ�� vx/vy=1673/647=2.6;

tim5_ch1������ʼ��
tim5_ch2�����˶���̬
tim_ch3����ģʽ

lowest 120000
middle 200000

highest 280000


�����õĴ����������������Ҷ�Ѳ�ߡ��������������ǡ����ٶȼơ�������

		if(code_record==1&&CAR_TURE&&order_step==0)//�������ñ��������ƵĴ��룬����˼·���Բο����żܵĴ���
		{
			
			PID_calc(&motor_move_displace_pid[0],motor[0].change,order[order_step].displacement);		
			car.vx=motor_move_displace_pid[0].out;
			car.vy=car.vx*order[order_step].rata;
			move_pid_calc();
      if(motor[0].change==order[order_step].displacement)	order_step++;
		}
		else if(order_step==1&&CAR_TURE)
		{
			car.vx=0;
			car.vy=0;
			car.w=0.5;
			while(!(rx_line_buff[1]==0xFB&&rx_line_buff[3]==0xDF))
//				(!(((rx_line_buff[0]&0x01)&&(rx_line_buff[2]&0x08))||((rx_line_buff[0]&0x04)&&(rx_line_buff[2]&0x20))||((rx_line_buff[0]&0x10)&&(rx_line_buff[2]&0x80))))
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			car.w=0;
			car.vx=20;
			car.vy=0;
			while(echo_distance>700)
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=0;
			car.vy=10;
			while(echo_distance>150)
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			frame_high=120000;
			spi_tx_buff[1]=0xF0;
			car.vx=10;
			car.vy=0;
			while(!(spi_rx_buff[1]==0xF0))
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=0;
			while(!(spi_rx_buff[1]==0xFA))
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=10;
			while(!(spi_rx_buff[1]==0xF0))
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
		}
		else if(code_record==0&&CAR_TURE)//��������дͨ���ٶȿ��ƵĴ���
		{	
			
		}