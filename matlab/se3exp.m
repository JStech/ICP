function [ SE3 ] = se3_SE3( se3 )
  w=se3(4:6)';
  u=se3(1:3)';
  wx=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
  theta=sqrt(w'*w);
  if(theta~=0),
      A=sin(theta)/theta;
      B=(1-cos(theta))/(theta^2);
      C=(1-A)/(theta^2);
  else
      A=0;
      B=0;
      C=0;
  end
  R=eye(3)+(A*wx)+(B*(wx*wx));
  V=eye(3)+B*wx+C*(wx*wx);
  Vp=V*u;
  SE3=zeros(4);
  SE3(1:3,1:3)=R;
  SE3(1:3,4)=Vp;
  SE3(4,4)=1;
end
