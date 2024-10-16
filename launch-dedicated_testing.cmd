:: example launch script, see https://github.com/OpenRA/OpenRA/wiki/Dedicated for details

@echo on

set Name="test"
set Mod=ra
set ListenPort=1234
set AdvertiseOnline=False
set Password="gdp"
set RecordReplays=False

set RequireAuthentication=False
set ProfileIDBlacklist=""
set ProfileIDWhitelist=""

set EnableSingleplayer=False
set EnableSyncReports=True
set EnableGeoIP=True
set EnableLintChecks=True
set ShareAnonymizedIPs=True

set JoinChatDelay=5000

set SupportDir=""

:loop

bin\OpenRA.Server.exe Engine.EngineDir=".." Game.Mod=%Mod% Server.Name=%Name% Server.ListenPort=%ListenPort% Server.AdvertiseOnline=%AdvertiseOnline% Server.EnableSingleplayer=%EnableSingleplayer% Server.Password=%Password% Server.RecordReplays=%RecordReplays% Server.RequireAuthentication=%RequireAuthentication% Server.ProfileIDBlacklist=%ProfileIDBlacklist% Server.ProfileIDWhitelist=%ProfileIDWhitelist% Server.EnableSyncReports=%EnableSyncReports% Server.EnableGeoIP=%EnableGeoIP% Server.EnableLintChecks=%EnableLintChecks% Server.ShareAnonymizedIPs=%ShareAnonymizedIPs% Server.JoinChatDelay=%JoinChatDelay% Engine.SupportDir=%SupportDir%

goto loop
