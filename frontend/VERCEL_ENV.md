# Vercel Deployment - Environment Variables

Add these environment variables in your Vercel project settings:

## Dashboard → Project → Settings → Environment Variables

### Production Environment

```
VITE_USE_CLOUD_MODE=true
VITE_CLOUD_BACKEND_URL=wss://your-app.onrender.com/ui
VITE_ROBOT_ID=mentorpi
```

### Preview Environment (optional)

```
VITE_USE_CLOUD_MODE=true
VITE_CLOUD_BACKEND_URL=wss://your-app-staging.onrender.com/ui
VITE_ROBOT_ID=mentorpi-dev
```

## Deploy via Vercel CLI

```bash
cd frontend
vercel --prod
```

## Auto-Deploy from Git

1. Import project on vercel.com
2. Connect GitHub repository
3. Set environment variables
4. Deploy!

Every push to main branch will auto-deploy.
