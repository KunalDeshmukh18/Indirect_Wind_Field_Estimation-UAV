using LinearAlgebra
using SymEngine
function hat(v)
    A=[0    -v[3]  v[2];
        v[3]  0    -v[1];
        -v[2]  v[1]  0];
    return A
end

function random_rotmat(complex=false)
    if complex
        temp=rand(ComplexF64,3)
        random_magnitude=pi*rand(ComplexF64)
    else
        temp=rand(Float64,3)
        random_magnitude=pi*rand(Float64)
    end
    temp_norm=temp'*temp
    random_axis=temp/temp_norm
    random_rotvec=random_magnitude*random_axis
    return exp(hat(random_rotvec));
end

function subs_SymEngineArrayExpr(expr,vars,variable_vals)
    out=Array{Basic}(undef,size(expr)...)
        for idx in eachindex(expr)
           out[idx]=subs(expr[idx],[vars[i]=>variable_vals[i] for i in eachindex(vars)]...)
        end
    return out;
end

function eval_SymEngineMatrixExpr(::Type{T},expr,vars,variable_vals) where {T}
    out=Matrix{T}(undef,size(expr)...)
    for idx in eachindex(expr)
        out[idx]=N(subs(expr[idx],[vars[i]=>variable_vals[i] for i in eachindex(vars)]...))
    end
    return out;
end