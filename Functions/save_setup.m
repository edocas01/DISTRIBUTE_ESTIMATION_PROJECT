function save_setup(R,T,O,LO,u_traj,param)
	if not(isfolder(param.Setup_folder))
		mkdir(param.Setup_folder)
	end
	name = [param.Setup_folder,'/','variables','.mat'];
    save(name, "R");
	save(name, "T", "-append");
	save(name, "O", "-append");
	save(name, "LO", "-append");
	save(name, "u_traj", "-append");
end