%����Ⱥ�㷨����A�������˻�ģ��
%B���Ը���A��typeֱ��ѡ�񣬻��߼���A�ĳ�ʼλ�úͽǶȣ���������磬ģ����ѡ����ʵ�Ƿֶκ���:D��������¥
%                             _ooOoo_
%                            o8888888o
%                            88" . "88
%                            (| -_- |)
%                             O\ = /O
%                         ____/`---'\____
%                       .   ' \\| |// `.
%                        / \\||| 8 |||// \
%                      / _||||| -8- |||||- \
%                        | | \\\ 8 /// | |
%                      | \_| ''\-8-/'' | |
%                       \ .-\__ `8` ___/-. /
%                    ___`. .' /--8--\ `. . __
%                 ."" '< `.___\_<8>_/___.' >'"".
%                | | : `- \`.;`\ 8 /`;.`/ - ` : | |
%                  \ \ `-. \_ __\ /__ _/ .-` / /
%          ======`-.____`-.___\_____/___.-`____.-'======
%                             `=---='
% 
%          .............................................
%                   ���汣��                  ����BUG
%           ��Ի:
%                   д��¥��д�ּ䣬д�ּ������Ա��
%                   ������Աд�������ó��򻻾�Ǯ��
%                   ����ֻ���������������������ߣ�
%                   ��������ո��գ����������긴�ꡣ
%                   ��Ը�������Լ䣬��Ը�Ϲ��ϰ�ǰ��
%                   ���۱������Ȥ���������г���Ա��
%                   ����Ц��߯��񲣬��Ц���˿�ëƬ��
%                   ��������Ư���ã��ĸ���ó���Ա��
function air_fun
close all;clear all;clc;
figure('Color','w');
% axis([0,50000,0,50000]);
%B����Ϸ���ݴ洢(����A��type����ʼλ�úͳ�ʼ�Ƕ�������(��������û��)��position_B��angler_B�����������������)
%������ֻҪBӮ�����ݣ�matlab ��nntool����ʵ����һĿ�꣬
%�ο�����https://blog.csdn.net/dftxmloo600965/article/details/101633876?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.edu_weight&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.edu_weight
global position_B angler_B type_A_save
position_B=[];
angler_B=[];
type_A=1;
%Bָ����
global pbq;
% ������Ϣ
x_scale=[0,50000];
y_scale=[0,50000];
% Сָ����Ʒ�Χ
x_sub_scale=500;
y_sub_scale=500;
% ���񾫶�m
step=50;
[X,Y]=meshgrid(x_scale(1):step:x_scale(2),y_scale(1):step:y_scale(2));
%ͼ��fps����s/ʱ�侫��s
time_step=2;
show_time_step=1;
% �������˻�����
global posB angB speed_lgB speed_agB;
%λ��/�Ƕ�
posB=[0,25000];
angB=0;
%0 ������1��ת��2��ת
angB_type=0;
%��ʱ�䲽�ƶ�����
speed_lgB=250*time_step;
% speed_agB=90/((200*2*pi)/4/250)*time_step;
%��ʱ�䲽�ƶ��Ƕȣ�a��d���ƣ�һ��5��
speed_agB=0;
speed_lgB_max=250*time_step;
%����ת�Ƕ�
agB_max=250;
speed_agB_max=90/((agB_max*2*pi)/4/250)*time_step;
%ָ����Ʒ�Χ
show_lgB=5000;

% �������˻�����
global posA angA speed_lgA speed_agA;
% λ��/�Ƕ�
posA={[100,100],[100,100],[100,100],[100,100],[100,100]};
angA={180,180,180,180,180};
% ����5ѡ2�б������ж�A����������A�ľ����Լ�B����������A�ľ���
choose2list=nchoosek(1:5,2);
%��λʱ�����ʱ��
speed_lgA_max=250*time_step;
%����ת�Ƕ�
agA_max=200;
speed_agA_max=90/((agA_max*2*pi)/4/250)*time_step;
speed_agA_max_pi=speed_agA_max/180*pi;
%��ǰ��ʽ�ٶȺͻ�ת�ٶ�
speed_lgA={speed_lgA_max,speed_lgA_max,speed_lgA_max,speed_lgA_max,speed_lgA_max};
speed_agA={speed_agA_max,speed_agA_max,speed_agA_max,speed_agA_max,speed_agA_max};

% �����С��Χ�Լ�ΧŹ����
A_min_lg=300;
A_max_lg=2000;
A_attack_lg=300+speed_lgA_max;


%% ��Ⱥ�㷨����
%ͷ��λ��
head_woft_X=0;
head_woft_Y=0;
%ΧŹ����
A_weiou_lg=3000;
%״̬���أ�1��B�Ƿ������2��B�Ƿ�ͻ�ƣ�3��B�Ƿ�ﵽΧŹ����
state_key=[0,0,0,0,0,0];


%% ���򲿷�
%��ʼ��
set(gcf,'KeyPressFcn',@shoot_by_key);
time_per=show_time_step;
air_A_init(40000,25000,100);
game = timer('ExecutionMode', 'FixedRate', 'Period',time_per, 'TimerFcn', @main_loop);
set(gcf,'tag','co','CloseRequestFcn',@clo);
    function clo(~,~),stop(game),delete(findobj('tag','co'));clf,close,end
start(game)
    %������ѭ��
    function main_loop(~,~)
        %״̬�ռ�
        state();
        if state_key(1)==1
            stop(game);
            disp('���ػ�ʤ��');
        end
        if state_key(2)==1
            stop(game);
            disp('������ʤ��');
        end
        
        n=size(posA);
        % AI���ܷ���
        if state_key(3)==0
            for i=1:n(2)
                if i==1
                    air_plan(i,1);
                else
                    air_plan(i,2);
                end
            end
        else
            for i=1:n(2)
                air_plan(i,3);
            end
        end
        %ͼ1��������λ��
        subplot(2,1,1);
        for i=1:n(2)
            airdef_draw(i,5000);
        end
        airgame;
        axis([x_scale(1),x_scale(2),y_scale(1),y_scale(2)]);
        hold off;
        %ͼ2���Ƽ�Ⱥ1����λ��
        subplot(2,1,2);
        for i=1:n(2)
            airdef_draw(i,100);
        end
        airgame;
        axis([posA{1}(1)-x_sub_scale,posA{1}(1)+x_sub_scale,...
            posA{1}(2)-y_sub_scale,posA{1}(2)+y_sub_scale]);
        %�洢���������ݺͷ��ػ�����
        position_B=[position_B,[posB(1);posB(2)]];
        angler_B=[angler_B,angB];
        type_A_save=type_A;
        hold off;
        drawnow;
    end
%λ�ýǶȳ�ʼ����AΪԲ�Σ�
    function air_A_init(x,y,r)
        n=size(posA);
        for i=1:n(2)
            theta=2*pi*i/n(2);
            posA{i}(1)=x+r*cos(theta);
            posA{i}(2)=y+r*sin(theta);
        end
    end
%% ��Ⱥ�㷨:1,ͷ����Ŀ��ǰ����2�����Ǹ���ͷ�ǣ�3���ж����Χ��
    function air_plan(i,type)
        if type==1
            % ͷ��
            x_target=posB(1);
            y_target=posB(2);
            head_woft_X=x_target;
            head_woft_Y=y_target;
            move_to_target(i,x_target,y_target);
        elseif type==2
            % ����
            x_target=head_woft_X;
            y_target=head_woft_Y;
            move_to_target_without_angle(i,x_target,y_target)
        elseif type==3
            % Χ��
            x_target=posB(1);
            y_target=posB(2);
            move_to_target(i,x_target,y_target);
        else
            warning('ERROR input');
        end
        
    end
%% ����ж����
    function state()
        %�ж��Ƿ������������˻�ΧŹ
        Z_fly=cir_block(posB(1),posB(2),A_attack_lg);
        Z_range_tmp=zeros(size(Z_fly));
        for i_last=1:size(choose2list,1)
            Z_1=cir_block(posA{choose2list(i_last,1)}(1),posA{choose2list(i_last,1)}(2),A_attack_lg);
            Z_2=cir_block(posA{choose2list(i_last,1)}(1),posA{choose2list(i_last,1)}(2),A_attack_lg);
            Z_range_tmp=Z_range_tmp+Z_1.*Z_2;
        end
        Z_range=Z_fly.*Z_range_tmp;
        x=find(Z_range(:)~=0);
        if size(x,1)~=0
            state_key(1)=1;
        end
        %�ж��Ƿ�ͻ��
        if posB(1)>x_scale(2)
            state_key(2)=1;
        else
            state_key(2)=0;
        end
        %�ж��Ƿ�ﵽΧŹ����
        if (head_woft_X-posB(1))^2+(head_woft_Y-posB(2))^2<A_weiou_lg^2
            state_key(3)=1;
        else
            state_key(3)=0;
        end
    end
%% ����������Ԫ
    function move_to_target_without_angle(i,x_target,y_target)
        %���Ǹ���ͷ���ƶ�ʱ���ƶ�����Ͻ�������Ҫ���ǽǶ�    
        [X0_pos,Y0_pos]=find_nearest(i,x_target,y_target);
        posA{i}(1)=X0_pos;
        posA{i}(2)=Y0_pos;
    end
    %ΧŹ��ͷ���ƶ�ʱ����Ҫ���ǽǶ�����
    function move_to_target(i,x_target,y_target)
        % ������򲻶ԣ��ȶ�׼����
        dx=posA{i}(1)-x_target;
        dy=y_target-posA{i}(2);
        if dx>0
            angler_target=180-atan(dy/dx)/pi*180;
        else
            angler_target=atan(dy/dx)/pi*180;
        end
        if abs(angler_target-angA{i})<speed_agA_max
            angA{i}=angler_target;
            % �ڷ���û���������£�����ת���Ƕȣ��������λ�ã�
            [X0_pos,Y0_pos]=find_nearest(i,x_target,y_target);
            posA{i}(1)=X0_pos;
            posA{i}(2)=Y0_pos;
        elseif angA{i}>angler_target
            angA{i}=angA{i}-speed_agA_max;
            x_tmp=agA_max*sin(speed_agA_max_pi);
            y_tmp=agA_max*(1-cos(speed_agA_max_pi));
            posA{i}(1)=posA{i}(1)+x_tmp*sin(angA{i});
            posA{i}(2)=posA{i}(2)-y_tmp*cos(angA{i});
        elseif angA{i}<angler_target
            angA{i}=angA{i}+speed_agA_max;
            x_tmp=agA_max*sin(speed_agA_max_pi);
            y_tmp=agA_max*(1-cos(speed_agA_max_pi));
            posA{i}(1)=posA{i}(1)+x_tmp*sin(angA{i});
            posA{i}(2)=posA{i}(2)-y_tmp*cos(angA{i});
        end
    end
    %������Լ������������£�Ѱ������·����������Ϊ�ǰ�˳���ƶ������1��2�����ܻ���ֱ������Ըľ��ȣ�Ҳ���Լ�ѭ��
    %̫�鷳��û�� :D
    function [X_pos,Y_pos]=find_nearest(i,x_target,y_target)
        Z_range=look_block(posA{i}(1),posA{i}(2),agA_max,angA{i});
        Z_fly=cir_block(posA{i}(1),posA{i}(2),speed_lgA{i});
        Z_range=Z_range.*Z_fly;
        Z_range_tmp=zeros(size(Z_range));
        for i_last=1:size(choose2list,1)
            Z_1=cir_block(posA{choose2list(i_last,1)}(1),posA{choose2list(i_last,1)}(2),A_max_lg);
            Z_2=(cir_block(posA{choose2list(i_last,1)}(1),posA{choose2list(i_last,1)}(2),A_min_lg)==0);
            Z_3=cir_block(posA{choose2list(i_last,2)}(1),posA{choose2list(i_last,2)}(2),A_max_lg);
            Z_4=(cir_block(posA{choose2list(i_last,2)}(1),posA{choose2list(i_last,2)}(2),A_min_lg)==0);
            Z_range_tmp=Z_range_tmp+Z_1.*Z_2.*Z_3.*Z_4;
        end
        Z_range=Z_range.*Z_range_tmp;
        Z_range=Z_range>0;
        X0=X.*Z_range;
        Y0=Y.*Z_range;
        xy_tmp=(X0-x_target).*Z_range+9000000*(~Z_range)+(Y0-y_target).*Z_range+9000000*(~Z_range);
        [xy_tmp,X0_pos]=min(xy_tmp);
        [~,Y0_pos]=min(xy_tmp);
        X0_pos=X0_pos(Y0_pos);
        X_pos=X(X0_pos,Y0_pos);
        Y_pos=Y(X0_pos,Y0_pos);
        if X0_pos==1&&Y0_pos==1
            warning('����ľ��ȣ�����������򾫶Ȼ򽵵�ʱ�侫��');
        end
    end
    % ��������
    function Z=cir_block(x_pos,y_pos,r)
        Z=(X-x_pos).^2+(Y-y_pos).^2-r^2;
        Z=Z<0;
    end
    %�Ƕ�����
    function Z=look_block(x_pos,y_pos,r,angler)
        angler=pi*angler/180;
        Z1=(X-(x_pos+r*sin(angler))).^2+(Y-(y_pos-r*cos(angler))).^2-r^2;
        Z1=Z1>0;
        Z2=(X-(x_pos-r*sin(angler))).^2+(Y-(y_pos+r*cos(angler))).^2-r^2;
        Z2=Z2>0;
        Z3=(X-x_pos)*cos(angler)+(Y-y_pos)*sin(angler);
        Z3=Z3>0;
        Z=Z1.*Z2.*Z3;
    end
    %���Ʒ�����A
    function airdef_draw(i,head_size)
        quiver(posA{i}(1),posA{i}(2),head_size*cos(angA{i}/180*pi),head_size*sin(angA{i}/180*pi),'LineWidth',1,...
            'MaxHeadSize',8,'Color','r');
        hold on;
    end
    %���Ʒ�����B
    function airgame
        posB(1)=posB(1)+speed_lgB*cos(angB/180*pi);
        posB(2)=posB(2)+speed_lgB*sin(angB/180*pi);
        pbq=quiver(posB(1),posB(2),show_lgB*cos(angB/180*pi),show_lgB*sin(angB/180*pi),'LineWidth',1,...
            'MaxHeadSize',8,'Color','b');
        hold on;
        draw_cir( posB(1),posB(2),A_attack_lg,'g' );
        switch angB_type
            case 1
                angB=angB+speed_agB;
            case 2
                angB=angB+speed_agB;
        end
        angB_type=0;
        speed_agB=0;
    end
    %ָ��λ�û�Ȧ
    function draw_cir( x,y,r,color )
        theta=0:0.1:2.5*pi;
        Circle1=x+r*cos(theta);
        Circle2=y+r*sin(theta);
        plot(Circle1,Circle2,'Color',color,'linewidth',1);
        axis equal
    end
    %��������
    function shoot_by_key(~,event)
        angB_type=0;
        switch event.Key
            case 'a'
                angB_type=1;
                speed_agB=speed_agB+5;
            case 'd'
                angB_type=2;
                speed_agB=speed_agB-5;
            otherwise
                angB_type=0;
        if speed_agB>speed_agB_max
            speed_agB=speed_agB_max;
        end
        if speed_agB<-speed_agB_max
            speed_agB=-speed_agB_max;
        end
        switch angB_type
            case 1
                angBtmp=angB+speed_agB;
            case 2
                angBtmp=angB+speed_agB;
            otherwise
                angBtmp=angB;
        end
        delete(pbq);
        pbq=quiver(posB(1),posB(2),show_lgB*cos(angBtmp/180*pi),show_lgB*sin(angBtmp/180*pi),'LineWidth',1,...
            'MaxHeadSize',8,'Color','b');
        drawnow;
        end
    end
end