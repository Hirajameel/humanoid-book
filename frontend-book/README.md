# Website

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

### For Vercel Deployment
The site is configured for Vercel deployment. Simply connect your GitHub repository to Vercel and the site will be automatically deployed.

### For GitHub Pages Deployment
The site is configured with a GitHub Actions workflow for automatic deployment to GitHub Pages. When you push to the main branch, the workflow will automatically build and deploy the site to GitHub Pages.

If you need to deploy manually to GitHub Pages, you can set the appropriate environment variables:

```bash
DEPLOYMENT_URL=https://humanoid-book.github.io BASE_URL=/humanoid-book/ yarn build
```

Then deploy the contents of the `build` folder to your GitHub Pages branch.
