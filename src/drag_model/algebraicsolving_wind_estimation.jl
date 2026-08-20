include("helper.jl")
using AlgebraicSolving

# Convert Float64 coefficients to exact Rationals for AbstractAlgebra...this is probably not a great long term solution as will produce large integers
Base.:*(a::MPolyRingElem, b::Float64) = a * rationalize(b)
Base.:*(a::Float64, b::MPolyRingElem) = rationalize(a) * b
Base.:+(a::MPolyRingElem, b::Float64) = a + rationalize(b)
Base.:+(a::Float64, b::MPolyRingElem) = rationalize(a) + b
Base.:-(a::MPolyRingElem, b::Float64) = a - rationalize(b)
Base.:-(a::Float64, b::MPolyRingElem) = rationalize(a) - b
Base.:^(a::MPolyRingElem, b::Float64) = a^Int(b)

timesteps=4#must also update the @vars calls
@vars m g R[1:4,1:3,1:3] Cd[1:3,1:3] u[1:4,1:4] v[1:4,1:3] a_o[1:4,1:3] w[1:4,1:3] Σ⁻[1:3,1:3]
Bf=[0 0 0 0;0 0 0 0;1 1 1 1];
gravity=[0;0;-g];
relative_velocity=v-w;
airspeed=sqrt.(sum(relative_velocity.*relative_velocity,dims=2));
wind_changes=w[2:timesteps,:]-w[1:timesteps-1,:];
a_predicted=stack([gravity+(R[i,:,:]*Bf*u[i,:]-R[i,:,:]*Cd*transpose(R[i,:,:])*airspeed[i]*relative_velocity[i,:])/m for i in 1:timesteps],dims=1);

error=a_o-a_predicted
wind_change_losses=sum(wind_changes.*(wind_changes*transpose(Σ⁻)),dims=2)#1 fewer than there are timesteps
loss=sum(error.*error,dims=2)
total_loss=sum(loss)+sum(wind_change_losses);

constant_term=stack([a_o[i,:]-gravity-R[i,:,:]*Bf*u[i,:]/m for i in 1:timesteps],dims=1)
@vars s[1:4]#airspeed
I3=[1 0 0;0 1 0;0 0 1];
s_times_accel_loss_deriv=stack([transpose(transpose(-2/m*R[i,:,:]*Cd*transpose(R[i,:,:])*s[i]*relative_velocity[i,:]-2*constant_term[i,:]/m)*R[i,:,:]*Cd*transpose(R[i,:,:])*(relative_velocity[i,:]*transpose(relative_velocity[i,:])+s[i]^2*I3)) for i in 1:timesteps],dims=1)
sdLdw=Matrix{Basic}(undef,timesteps,3)
sdLdw.=s_times_accel_loss_deriv;
for wind_idx in 1:timesteps
    #each wind shows up in change ids wind_idx-1, wind_idx,wind_idx+1 if all present
    if wind_idx-1>0
        from_previous=diff.(wind_change_losses[wind_idx-1],w[wind_idx,:])
        sdLdw[wind_idx,:]+=s[wind_idx]*from_previous
    end
    if wind_idx<timesteps
        from_current=diff.(wind_change_losses[wind_idx],w[wind_idx,:])
        sdLdw[wind_idx,:]+=s[wind_idx]*from_current
    end
    if wind_idx<timesteps-1
        from_next=diff.(wind_change_losses[wind_idx+1],w[wind_idx,:])
        sdLdw[wind_idx,:]+=s[wind_idx]*from_next
    end
end
# dLdw=differentiate(loss,w);
# dLdw_subs=subs(dLdw,airspeed=>s)
airspeed_constraint=[s[i]^2-relative_velocity[i,:]'*relative_velocity[i,:] for i in 1:timesteps]
system_state_vec=[w[1,:];s[1];w[2,:];s[2];w[3,:];s[3];w[4,:];s[4]]
system_param_vec=[m;g;vec(Cd);vec(a_o);vec(u);vec(R);vec(v);vec(Σ⁻)];
equations=vcat([[sdLdw[i,:];airspeed_constraint[i]] for i in 1:timesteps]...);

#generic parameter values

m_val=1#pick mass units appropriately
g_val=1#pick time/length units appropriately
v_val=randn(timesteps,3)
u_val=randn(timesteps,4)
Cd_val=rand(3,3)
w_true=randn(timesteps,3)
R_val=stack([random_rotmat(false) for i in 1:timesteps],dims=1);
sensor_error=randn(timesteps,3)*0.1;
a_true=eval_SymEngineMatrixExpr(Float64,a_predicted,[m;g;vec(Cd);vec(u);vec(R);vec(v);vec(w)],[m_val;g_val;vec(Cd_val);vec(u_val);vec(R_val);vec(v_val);vec(w_true)]);
a_val=sensor_error+a_true;
random_3by3=randn(3,3);
sigma_val=random_3by3*random_3by3'+I
random_param_vec=[m_val;g_val;vec(Cd_val);vec(a_val);vec(u_val);vec(R_val);vec(v_val);vec(sigma_val)]
# rational_parameters=rationalize.(random_param_vec) rationalizing before passing to symengine is substantially slower.
eqns_random_instance=subs_SymEngineArrayExpr(equations,system_param_vec,random_param_vec);
#setup AlgebraicSolving
ring,as_vars=polynomial_ring(QQ,vcat([vcat(["w$(i)_$(j)" for j in 1:3], ["s$(i)"]) for i in 1:timesteps]...))
flambda=lambdify(eqns_random_instance,system_state_vec)
ideal=Ideal(flambda(as_vars...));