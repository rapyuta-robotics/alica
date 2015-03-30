#Example of the Globals.conf

```
[Globals]
	//Every ROBOT should be registered here
	[Team]
		[odroid]
			ID = 134
			FollowStreet = Street
		[!odroid]

	[!Team]
	//Role matching is shown in the planDesigner tutorial
	[RolePriority]
		Drive = 1
	[!RolePriority]
[!Globals]

```
