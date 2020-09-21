%含狼群算法（仅A）的无人机模拟
%B可以根据A的type直接选择，或者加上A的初始位置和角度，算个神经网络，模型任选，其实是分段函数:D，佛祖镇楼
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
%                   佛祖保佑                  永无BUG
%           佛曰:
%                   写字楼里写字间，写字间里程序员；
%                   程序人员写程序，又拿程序换酒钱。
%                   酒醒只在网上坐，酒醉还来网下眠；
%                   酒醉酒醒日复日，网上网下年复年。
%                   但愿老死电脑间，不愿鞠躬老板前；
%                   奔驰宝马贵者趣，公交自行程序员。
%                   别人笑我忒疯癫，我笑别人看毛片；
%                   不见满街漂亮妹，哪个归得程序员？
function air_fun
close all;clear all;clc;
figure('Color','w');
% axis([0,50000,0,50000]);
%B的游戏数据存储(利用A的type，初始位置和初始角度做输入(后两个还没加)；position_B，angler_B做输出，构建神经网络)
%！！！只要B赢的数据，matlab 的nntool可以实现这一目标，
%参考资料https://blog.csdn.net/dftxmloo600965/article/details/101633876?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.edu_weight&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.edu_weight
global position_B angler_B type_A_save
position_B=[];
angler_B=[];
type_A=1;
%B指针句柄
global pbq;
% 场地信息
x_scale=[0,50000];
y_scale=[0,50000];
% 小指针绘制范围
x_sub_scale=500;
y_sub_scale=500;
% 网格精度m
step=50;
[X,Y]=meshgrid(x_scale(1):step:x_scale(2),y_scale(1):step:y_scale(2));
%图像fps精度s/时间精度s
time_step=2;
show_time_step=1;
% 进攻无人机参数
global posB angB speed_lgB speed_agB;
%位置/角度
posB=[0,25000];
angB=0;
%0 不动；1左转；2右转
angB_type=0;
%单时间步移动距离
speed_lgB=250*time_step;
% speed_agB=90/((200*2*pi)/4/250)*time_step;
%单时间步移动角度，a和d控制，一次5度
speed_agB=0;
speed_lgB_max=250*time_step;
%最大回转角度
agB_max=250;
speed_agB_max=90/((agB_max*2*pi)/4/250)*time_step;
%指针绘制范围
show_lgB=5000;

% 防御无人机参数
global posA angA speed_lgA speed_agA;
% 位置/角度
posA={[100,100],[100,100],[100,100],[100,100],[100,100]};
angA={180,180,180,180,180};
% 生成5选2列表，用于判断A与任意两个A的距离以及B与任意两个A的距离
choose2list=nchoosek(1:5,2);
%单位时间最大时速
speed_lgA_max=250*time_step;
%最大回转角度
agA_max=200;
speed_agA_max=90/((agA_max*2*pi)/4/250)*time_step;
speed_agA_max_pi=speed_agA_max/180*pi;
%当前形式速度和回转速度
speed_lgA={speed_lgA_max,speed_lgA_max,speed_lgA_max,speed_lgA_max,speed_lgA_max};
speed_agA={speed_agA_max,speed_agA_max,speed_agA_max,speed_agA_max,speed_agA_max};

% 最大最小范围以及围殴距离
A_min_lg=300;
A_max_lg=2000;
A_attack_lg=300+speed_lgA_max;


%% 狼群算法参数
%头狼位置
head_woft_X=0;
head_woft_Y=0;
%围殴距离
A_weiou_lg=3000;
%状态开关：1，B是否过近；2，B是否突破；3，B是否达到围殴距离
state_key=[0,0,0,0,0,0];


%% 程序部分
%初始化
set(gcf,'KeyPressFcn',@shoot_by_key);
time_per=show_time_step;
air_A_init(40000,25000,100);
game = timer('ExecutionMode', 'FixedRate', 'Period',time_per, 'TimerFcn', @main_loop);
set(gcf,'tag','co','CloseRequestFcn',@clo);
    function clo(~,~),stop(game),delete(findobj('tag','co'));clf,close,end
start(game)
    %程序主循环
    function main_loop(~,~)
        %状态收集
        state();
        if state_key(1)==1
            stop(game);
            disp('防守机胜利');
        end
        if state_key(2)==1
            stop(game);
            disp('进攻机胜利');
        end
        
        n=size(posA);
        % AI智能方案
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
        %图1绘制总体位置
        subplot(2,1,1);
        for i=1:n(2)
            airdef_draw(i,5000);
        end
        airgame;
        axis([x_scale(1),x_scale(2),y_scale(1),y_scale(2)]);
        hold off;
        %图2绘制集群1总体位置
        subplot(2,1,2);
        for i=1:n(2)
            airdef_draw(i,100);
        end
        airgame;
        axis([posA{1}(1)-x_sub_scale,posA{1}(1)+x_sub_scale,...
            posA{1}(2)-y_sub_scale,posA{1}(2)+y_sub_scale]);
        %存储进攻机数据和防守机类型
        position_B=[position_B,[posB(1);posB(2)]];
        angler_B=[angler_B,angB];
        type_A_save=type_A;
        hold off;
        drawnow;
    end
%位置角度初始化（A为圆形）
    function air_A_init(x,y,r)
        n=size(posA);
        for i=1:n(2)
            theta=2*pi*i/n(2);
            posA{i}(1)=x+r*cos(theta);
            posA{i}(2)=y+r*sin(theta);
        end
    end
%% 狼群算法:1,头狼向目标前进；2，猛狼跟随头狼；3，判断情况围攻
    function air_plan(i,type)
        if type==1
            % 头狼
            x_target=posB(1);
            y_target=posB(2);
            head_woft_X=x_target;
            head_woft_Y=y_target;
            move_to_target(i,x_target,y_target);
        elseif type==2
            % 猛狼
            x_target=head_woft_X;
            y_target=head_woft_Y;
            move_to_target_without_angle(i,x_target,y_target)
        elseif type==3
            % 围攻
            x_target=posB(1);
            y_target=posB(2);
            move_to_target(i,x_target,y_target);
        else
            warning('ERROR input');
        end
        
    end
%% 情况判断语句
    function state()
        %判断是否被两架以上无人机围殴
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
        %判断是否突防
        if posB(1)>x_scale(2)
            state_key(2)=1;
        else
            state_key(2)=0;
        end
        %判断是否达到围殴距离
        if (head_woft_X-posB(1))^2+(head_woft_Y-posB(2))^2<A_weiou_lg^2
            state_key(3)=1;
        else
            state_key(3)=0;
        end
    end
%% 基本函数单元
    function move_to_target_without_angle(i,x_target,y_target)
        %猛狼跟随头狼移动时，移动距离较近，不需要考虑角度    
        [X0_pos,Y0_pos]=find_nearest(i,x_target,y_target);
        posA{i}(1)=X0_pos;
        posA{i}(2)=Y0_pos;
    end
    %围殴或头狼移动时，需要考虑角度问题
    function move_to_target(i,x_target,y_target)
        % 如果方向不对，先对准方向，
        dx=posA{i}(1)-x_target;
        dy=y_target-posA{i}(2);
        if dx>0
            angler_target=180-atan(dy/dx)/pi*180;
        else
            angler_target=atan(dy/dx)/pi*180;
        end
        if abs(angler_target-angA{i})<speed_agA_max
            angA{i}=angler_target;
            % 在方向没问题的情况下，不再转动角度，到最近的位置，
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
    %在满足约束条件的情况下，寻找最优路径方案，因为是按顺序移动，最后1，2个可能会出现报错，可以改精度，也可以加循环
    %太麻烦就没加 :D
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
            warning('错误的精度，可以提高区域精度或降低时间精度');
        end
    end
    % 距离限制
    function Z=cir_block(x_pos,y_pos,r)
        Z=(X-x_pos).^2+(Y-y_pos).^2-r^2;
        Z=Z<0;
    end
    %角度限制
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
    %绘制飞行器A
    function airdef_draw(i,head_size)
        quiver(posA{i}(1),posA{i}(2),head_size*cos(angA{i}/180*pi),head_size*sin(angA{i}/180*pi),'LineWidth',1,...
            'MaxHeadSize',8,'Color','r');
        hold on;
    end
    %绘制飞行器B
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
    %指定位置画圈
    function draw_cir( x,y,r,color )
        theta=0:0.1:2.5*pi;
        Circle1=x+r*cos(theta);
        Circle2=y+r*sin(theta);
        plot(Circle1,Circle2,'Color',color,'linewidth',1);
        axis equal
    end
    %按键接收
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