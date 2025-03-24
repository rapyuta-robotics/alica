# Setup

To be able to login to github and use the Git-workflow to import and export plan, it is necessary to register the web-plan-designer application as an oauth2 client with github before launching the web-plan-designer.

1.Follow [Github's guide](https://docs.github.com/en/developers/apps/building-oauth-apps/creating-an-oauth-app) to create a new oauth application. Enter the URL of the plan designer (default: `http://localhost:3030/`) as the 'Homepage URL' and 'Authorization Callback URL'.

![oauth_register](../images/oauth_register.png)


2.Note down the client ID and client secret after registering the application in the previous step.

![oauth_creds](../images/oauth_creds.png)

3.Set the client ID and client secret as environment variables before launching the web-plan-designer

```
    export SOCIAL_APP_CLIENT_ID=<client_id>
    export SOCIAL_APP_SECRET=<client_secret
```

This can also be set in `config.env`

4.Set the client ID in the web-plan-designer [settings](#21-settings) after launch

![oauth_settings](../images/oauth_settings.png)